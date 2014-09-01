
#include <libssh2.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>

#include <iostream>
#include <string>
#include <fstream>

#include "SshConnection.hpp"


static int waitsocket(int socket_fd, LIBSSH2_SESSION *session_ssh_) {
  struct timeval timeout;
  int rc;
  fd_set fd;
  fd_set *writefd = NULL;
  fd_set *readfd = NULL;
  int dir;

  /*timeout definition */
  timeout.tv_sec = 0;
  timeout.tv_usec = 100;

  FD_ZERO(&fd);
  FD_SET(socket_fd, &fd);

  /* now make sure we wait in the correct direction */
  dir = libssh2_session_block_directions(session_ssh_);

  if (dir & LIBSSH2_SESSION_BLOCK_INBOUND)
    readfd = &fd;

  if (dir & LIBSSH2_SESSION_BLOCK_OUTBOUND)
    writefd = &fd;

  rc = select(socket_fd + 1, readfd, writefd, NULL, &timeout);

  return rc;
}

SshConnection::SshConnection(const std::string &hostname, const std::string &username, const std::string &password, const unsigned int port):
                             hostname_(hostname),
                             username_(username),
                             password_(password),
                             port_(port)
{

  /* init ssh session instance  */
  session_ssh_ = NULL;

  //establish connection to ssh server
  bool success = sshConnect();

  if(!success)
  {
    std::cout << "Error: could not connect to sensor at " << hostname_ << "\n";
    exit(1);
  }


}

SshConnection::~SshConnection()
{
  //disconnect from ssh
  sshDisconnect();
}


bool SshConnection::runCommand(const std::string &command, std::string &output, int &exitcode) const
{

  LIBSSH2_CHANNEL *channel;
  int rc;
  char *exitsignal=(char *)"none";
  int bytecount = 0;

  /*clear the output*/
  output.clear();

  /* Exec non-blocking on the remove host */
  while( (channel = libssh2_channel_open_session(session_ssh_)) == NULL && libssh2_session_last_error(session_ssh_,NULL,NULL,0) == LIBSSH2_ERROR_EAGAIN )
  {
      waitsocket(socket_, session_ssh_);
  }

  if( channel == NULL )
  {
      std::cout << "Error runCommand\n";
      return false;
  }
  while( (rc = libssh2_channel_exec(channel, command.c_str())) == LIBSSH2_ERROR_EAGAIN )
  {
      waitsocket(socket_, session_ssh_);
  }

  if( rc != 0 )
  {
      std::cout << "Error runCommand\n";
      return false;
  }
  for( ;; )
  {
      /* loop until we block */
      int rc;
      do
      {
          char buffer[0x4000];
          rc = libssh2_channel_read( channel, buffer, sizeof(buffer) );

          if( rc > 0 )
          {
              bytecount += rc;

              //write received data into output buffer
              for(int i=0; i<rc; i++)
                output += buffer[i];

          }
      }
      while( rc > 0 );

      /* this is due to blocking that would occur otherwise so we loop on
         this condition */
      if( rc == LIBSSH2_ERROR_EAGAIN )
      {
          waitsocket(socket_, session_ssh_);
      }
      else
          break;
  }
  exitcode = 127;
  while( (rc = libssh2_channel_close(channel)) == LIBSSH2_ERROR_EAGAIN )
      waitsocket(socket_, session_ssh_);

  if( rc == 0 )
  {
      exitcode = libssh2_channel_get_exit_status( channel );

      libssh2_channel_get_exit_signal(channel, &exitsignal, NULL, NULL, NULL, NULL, NULL);
  }

  libssh2_channel_free(channel);
  return true;
}


bool SshConnection::sendFile(const std::string &srcpath, std::string &scppath) const
{

  LIBSSH2_CHANNEL *channel;

  FILE *local;
  struct stat fileinfo;
  int rc;
  char mem[1024*100];
  size_t nread;
  char *ptr;
  long total = 0;
  size_t prev;

  /* open the local file */
  local = fopen(srcpath.c_str(), "rb");
  if (!local) {
      std::cout << "sendfile: can't find local file" << srcpath.c_str() << "\n";
      return -1;
  }

  stat(srcpath.c_str(), &fileinfo);


  /* Send a file via scp. The mode parameter must only have permissions! */
  do {
      channel = libssh2_scp_send(session_ssh_, scppath.c_str(), fileinfo.st_mode & 0777, (unsigned long)fileinfo.st_size);

      if ((!channel) && (libssh2_session_last_errno(session_ssh_) != LIBSSH2_ERROR_EAGAIN))
      {
          char *err_msg;

          libssh2_session_last_error(session_ssh_, &err_msg, NULL, 0);

          std::cout << "sendFile error:" << err_msg << "\n";
          return false;
      }
  } while (!channel);

  /* send file*/
  do {
      nread = fread(mem, 1, sizeof(mem), local);
      if (nread <= 0) {
          /* end of file */
          break;
      }
      ptr = mem;

      total += nread;

      prev = 0;
      do {
          while ((rc = libssh2_channel_write(channel, ptr, nread)) == LIBSSH2_ERROR_EAGAIN)
          {
              waitsocket(socket_, session_ssh_);
              prev = 0;
          }
          if (rc < 0) {
              std::cout << "sendFile: ERROR " << rc << "total "<<total<<" / " << (int)nread << " prev "<< (int)prev << "\n";
              break;
          }
          else {
              prev = nread;

              /* rc indicates how many bytes were written this time */
              nread -= rc;
              ptr += rc;
          }
      } while (nread);
  } while (!nread); /* only continue if nread was drained */

  return true;
}


bool SshConnection::getFile(const std::string &scppath, std::string &dstpath) const
{
  LIBSSH2_CHANNEL *channel;
  struct stat fileinfo;
  int total = 0;
  int spin = 0;
  off_t got = 0;

  /* open local file where we write to */
  std::ofstream dstFile;
  try
  {
    dstFile.open(dstpath.c_str());
  }
  catch (std::ofstream::failure e)
  {
    std::cout << "SSH download to file error - could not open local file: " << dstpath << "\n";
    return false;
  }

  /* Request a file via SCP */
  do {
    channel = libssh2_scp_recv(session_ssh_, scppath.c_str(), &fileinfo);

    if (!channel) {
      if (libssh2_session_last_errno(session_ssh_) != LIBSSH2_ERROR_EAGAIN) {
        char *err_msg;

        libssh2_session_last_error(session_ssh_, &err_msg, NULL, 0);
        std::cout << "getFile error: " << err_msg << "\n";
        return false;
      } else {
        waitsocket(socket_, session_ssh_);
      }
    }
  } while (!channel);


  /* read data */
  while (got < fileinfo.st_size) {
    // data buffer
    char mem[1024 * 24];
    int rc;

    do {
      int amount = sizeof(mem);

      if ((fileinfo.st_size - got) < amount) {
        amount = fileinfo.st_size - got;
      }

      /* loop until we block */
      rc = libssh2_channel_read(channel, mem, amount);
      if (rc > 0) {

        //write received data into output buffer
        for(int i=0; i<rc; i++)
         dstFile << mem[i];

        got += rc;
        total += rc;
      }
    } while (rc > 0);

    if ((rc == LIBSSH2_ERROR_EAGAIN) && (got < fileinfo.st_size)) {
      /* this is due to blocking that would occur otherwise
       so we loop on this condition */

      spin++;
      waitsocket(socket_, session_ssh_); /* now we wait */
      continue;
    }
    break;
  }


  /* clean up channel */
  libssh2_channel_free(channel);
  channel = NULL;

  /* close file */
  dstFile.close();

  return true;
}

void SshConnection::sshDisconnect() {

  libssh2_session_disconnect(session_ssh_, "Normal Shutdown, Thank you for playing");
  libssh2_session_free(session_ssh_);
  close(socket_);
  libssh2_exit();
}


bool SshConnection::sshConnect() {
  unsigned long hostaddr;
  struct sockaddr_in sin;
  int rc;

  /* set the ip address */
  hostaddr = inet_addr(hostname_.c_str());

  /* init the libssh2 library */
  rc = libssh2_init(0);
  if (rc != 0) {
    std::cout << "libssh2 initialization failed (" << rc << ")\n";
    return false;
  }

  /* create socket to connect to ip */
  socket_ = socket(AF_INET, SOCK_STREAM, 0);

  sin.sin_family = AF_INET;
  sin.sin_port = htons(port_);
  sin.sin_addr.s_addr = hostaddr;
  if (connect(socket_, (struct sockaddr*) (&sin), sizeof(struct sockaddr_in)) != 0)
  {
    //could not connect to hostname...
    return false;
  }

  /* Create a session_ssh_ instance */
  session_ssh_ = libssh2_session_init();
  if (!session_ssh_)
    return false;

  /* Since we have set non-blocking, tell libssh2 we are non-blocking */
  libssh2_session_set_blocking(session_ssh_, 0);


  /* ... start it up. This will trade welcome banners, exchange keys,
   * and setup crypto, compression, and MAC layers
   */
  while ((rc = libssh2_session_handshake(session_ssh_, socket_)) == LIBSSH2_ERROR_EAGAIN);
  if (rc) {
    std::cout << "Failure establishing SSH session_ssh_: " << rc << "\n";
    return false;
  }

  /* We could authenticate via password */
  while ((rc = libssh2_userauth_password(session_ssh_, username_.c_str(), password_.c_str())) == LIBSSH2_ERROR_EAGAIN);
  if (rc) {
    std::cout << "SSH authentication by password failed.\n";
    return false;
  }

  return true;
}

