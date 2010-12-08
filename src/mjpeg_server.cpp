/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "mjpeg_server/mjpeg_server.h"
#include <highgui.h>

#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syslog.h>
#include <netdb.h>
#include <errno.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define ABS(a) (((a) < 0) ? -(a) : (a))
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#define LENGTH_OF(x) (sizeof(x)/sizeof(x[0]))

namespace mjpeg_server {

MJPEGServer::MJPEGServer(ros::NodeHandle& node) :
  node_(node), image_transport_(node), stop_requested_(false), www_folder_(NULL)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("port", port_, 8080);
}

MJPEGServer::~MJPEGServer() {
  cleanUp();
}

void MJPEGServer::imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic) {
  IplImage *cv_image = NULL;
  try {
   if (bridge_.fromImage(*msg, "bgr8")) {
     cv_image = bridge_.toIpl();
   }
   else {
     ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
     return;
   }
  }
  catch(...) {
   ROS_ERROR("Unable to convert %s image to ipl format", msg->encoding.c_str());
   return;
  }

  ImageBuffer* image_buffer = image_buffers_[topic];

  // lock image buffer
  boost::unique_lock<boost::mutex> lock(image_buffer->mutex_);

  // encode image
  cv::Mat img = cv_image;
  std::vector<uchar> buffer;
  cv::imencode(".jpeg", img, buffer);

  int buffer_size = buffer.size();
  if(buffer_size == 0)
    return;

  // check if image buffer is large enough, increase it if necessary
  if(buffer_size > image_buffer->size_) {
    ROS_DEBUG("increasing buffer size to %d\n", buffer_size);
    image_buffer->buffer_ = (char*)realloc(image_buffer->buffer_, buffer_size);
    image_buffer->buffer_size_ = buffer_size;
  }

  // copy image buffer
  memcpy(image_buffer->buffer_, &buffer[0], buffer_size);
  image_buffer->size_ = buffer_size;
  image_buffer->time_stamp_ = msg->header.stamp.toSec();

  // notify senders
  image_buffer->condition_.notify_all();
}

/******************************************************************************
Description.: initializes the iobuffer structure properly
Input Value.: pointer to already allocated iobuffer
Return Value: iobuf
******************************************************************************/
void MJPEGServer::init_iobuffer(iobuffer *iobuf)
{
    memset(iobuf->buffer, 0, sizeof(iobuf->buffer));
    iobuf->level = 0;
}

/******************************************************************************
Description.: initializes the request structure properly
Input Value.: pointer to already allocated req
Return Value: req
******************************************************************************/
void MJPEGServer::init_request(request *req)
{
    req->type        = A_UNKNOWN;
    req->type        = A_UNKNOWN;
    req->parameter   = NULL;
    req->client      = NULL;
    req->credentials = NULL;
}

/******************************************************************************
Description.: If strings were assigned to the different members free them
              This will fail if strings are static, so always use strdup().
Input Value.: req: pointer to request structure
Return Value: -
******************************************************************************/
void MJPEGServer::free_request(request *req)
{
    if(req->parameter != NULL) free(req->parameter);
    if(req->client != NULL) free(req->client);
    if(req->credentials != NULL) free(req->credentials);
}

/******************************************************************************
Description.: read with timeout, implemented without using signals
              tries to read len bytes and returns if enough bytes were read
              or the timeout was triggered. In case of timeout the return
              value may differ from the requested bytes "len".
Input Value.: * fd.....: fildescriptor to read from
              * iobuf..: iobuffer that allows to use this functions from multiple
                         threads because the complete context is the iobuffer.
              * buffer.: The buffer to store values at, will be set to zero
                         before storing values.
              * len....: the length of buffer
              * timeout: seconds to wait for an answer
Return Value: * buffer.: will become filled with bytes read
              * iobuf..: May get altered to save the context for future calls.
              * func().: bytes copied to buffer or -1 in case of error
******************************************************************************/
int MJPEGServer::_read(int fd, iobuffer *iobuf, void *buffer, size_t len, int timeout)
{
    int copied = 0, rc, i;
    fd_set fds;
    struct timeval tv;

    memset(buffer, 0, len);

    while((copied < len)) {
        i = MIN(iobuf->level, len - copied);
        memcpy(buffer + copied, iobuf->buffer + IO_BUFFER - iobuf->level, i);

        iobuf->level -= i;
        copied += i;
        if(copied >= len)
            return copied;

        /* select will return in case of timeout or new data arrived */
        tv.tv_sec = timeout;
        tv.tv_usec = 0;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        if((rc = select(fd + 1, &fds, NULL, NULL, &tv)) <= 0) {
            if(rc < 0)
                exit(EXIT_FAILURE);

            /* this must be a timeout */
            return copied;
        }

        init_iobuffer(iobuf);

        /*
         * there should be at least one byte, because select signalled it.
         * But: It may happen (very seldomly), that the socket gets closed remotly between
         * the select() and the following read. That is the reason for not relying
         * on reading at least one byte.
         */
        if((iobuf->level = read(fd, &iobuf->buffer, IO_BUFFER)) <= 0) {
            /* an error occured */
            return -1;
        }

        /* align data to the end of the buffer if less than IO_BUFFER bytes were read */
        memmove(iobuf->buffer + (IO_BUFFER - iobuf->level), iobuf->buffer, iobuf->level);
    }

    return 0;
}

/******************************************************************************
Description.: Read a single line from the provided fildescriptor.
              This funtion will return under two conditions:
              * line end was reached
              * timeout occured
Input Value.: * fd.....: fildescriptor to read from
              * iobuf..: iobuffer that allows to use this functions from multiple
                         threads because the complete context is the iobuffer.
              * buffer.: The buffer to store values at, will be set to zero
                         before storing values.
              * len....: the length of buffer
              * timeout: seconds to wait for an answer
Return Value: * buffer.: will become filled with bytes read
              * iobuf..: May get altered to save the context for future calls.
              * func().: bytes copied to buffer or -1 in case of error
******************************************************************************/
/* read just a single line or timeout */
int MJPEGServer::_readline(int fd, iobuffer *iobuf, char *buffer, size_t len, int timeout)
{
    char c = '\0', *out = buffer;
    int i;

    memset(buffer, 0, len);

    for(i = 0; i < len && c != '\n'; i++) {
        if(_read(fd, iobuf, &c, 1, timeout) <= 0) {
            /* timeout or error occured */
            return -1;
        }
        *out++ = c;
    }

    return i;
}

/******************************************************************************
Description.: Decodes the data and stores the result to the same buffer.
              The buffer will be large enough, because base64 requires more
              space then plain text.
Hints.......: taken from busybox, but it is GPL code
Input Value.: base64 encoded data
Return Value: plain decoded data
******************************************************************************/
void MJPEGServer::decodeBase64(char *data)
{
    const unsigned char *in = (const unsigned char *)data;
    /* The decoded size will be at most 3/4 the size of the encoded */
    unsigned ch = 0;
    int i = 0;

    while(*in) {
        int t = *in++;

        if(t >= '0' && t <= '9')
            t = t - '0' + 52;
        else if(t >= 'A' && t <= 'Z')
            t = t - 'A';
        else if(t >= 'a' && t <= 'z')
            t = t - 'a' + 26;
        else if(t == '+')
            t = 62;
        else if(t == '/')
            t = 63;
        else if(t == '=')
            t = 0;
        else
            continue;

        ch = (ch << 6) | t;
        i++;
        if(i == 4) {
            *data++ = (char)(ch >> 16);
            *data++ = (char)(ch >> 8);
            *data++ = (char) ch;
            i = 0;
        }
    }
    *data = '\0';
}

/******************************************************************************
Description.: convert a hexadecimal ASCII character to integer
Input Value.: ASCII character
Return Value: corresponding value between 0 and 15, or -1 in case of error
******************************************************************************/
int MJPEGServer::hex_char_to_int(char in)
{
    if(in >= '0' && in <= '9')
        return in - '0';

    if(in >= 'a' && in <= 'f')
        return (in - 'a') + 10;

    if(in >= 'A' && in <= 'F')
        return (in - 'A') + 10;

    return -1;
}

/******************************************************************************
Description.: replace %XX with the character code it represents, URI
Input Value.: string to unescape
Return Value: 0 if everything is ok, -1 in case of error
******************************************************************************/
int MJPEGServer::unescape(char *string)
{
    char *source = string, *destination = string;
    int src, dst, length = strlen(string), rc;

    /* iterate over the string */
    for(dst = 0, src = 0; src < length; src++) {

        /* is it an escape character? */
        if(source[src] != '%') {
            /* no, so just go to the next character */
            destination[dst] = source[src];
            dst++;
            continue;
        }

        /* yes, it is an escaped character */

        /* check if there are enough characters */
        if(src + 2 > length) {
            return -1;
            break;
        }

        /* perform replacement of %## with the corresponding character */
        if((rc = hex_char_to_int(source[src+1])) == -1) return -1;
        destination[dst] = rc * 16;
        if((rc = hex_char_to_int(source[src+2])) == -1) return -1;
        destination[dst] += rc;

        /* advance pointers, here is the reason why the resulting string is shorter */
        dst++; src += 2;
    }

    /* ensure the string is properly finished with a null-character */
    destination[dst] = '\0';

    return 0;
}

void MJPEGServer::send_error(int fd, int which, char *message)
{
    char buffer[BUFFER_SIZE] = {0};

    if(which == 401) {
        sprintf(buffer, "HTTP/1.0 401 Unauthorized\r\n" \
                "Content-type: text/plain\r\n" \
                STD_HEADER \
                "WWW-Authenticate: Basic realm=\"MJPG-Streamer\"\r\n" \
                "\r\n" \
                "401: Not Authenticated!\r\n" \
                "%s", message);
    } else if(which == 404) {
        sprintf(buffer, "HTTP/1.0 404 Not Found\r\n" \
                "Content-type: text/plain\r\n" \
                STD_HEADER \
                "\r\n" \
                "404: Not Found!\r\n" \
                "%s", message);
    } else if(which == 500) {
        sprintf(buffer, "HTTP/1.0 500 Internal Server Error\r\n" \
                "Content-type: text/plain\r\n" \
                STD_HEADER \
                "\r\n" \
                "500: Internal Server Error!\r\n" \
                "%s", message);
    } else if(which == 400) {
        sprintf(buffer, "HTTP/1.0 400 Bad Request\r\n" \
                "Content-type: text/plain\r\n" \
                STD_HEADER \
                "\r\n" \
                "400: Not Found!\r\n" \
                "%s", message);
    } else {
        sprintf(buffer, "HTTP/1.0 501 Not Implemented\r\n" \
                "Content-type: text/plain\r\n" \
                STD_HEADER \
                "\r\n" \
                "501: Not Implemented!\r\n" \
                "%s", message);
    }

    if(write(fd, buffer, strlen(buffer)) < 0) {
        ROS_DEBUG("write failed, done anyway");
    }
}

/******************************************************************************
Description.: Send a complete HTTP response and a stream of JPG-frames.
Input Value.: fildescriptor fd to send the answer to
Return Value: -
******************************************************************************/
void MJPEGServer::send_stream(int fd, ImageBuffer* image_buffer)
{
    unsigned char *frame = NULL, *tmp = NULL;
    int frame_size = 0, max_frame_size = 0;
    char buffer[BUFFER_SIZE] = {0};
    struct timeval timestamp;

    ROS_DEBUG("preparing header\n");
    sprintf(buffer, "HTTP/1.0 200 OK\r\n" \
            STD_HEADER \
            "Content-Type: multipart/x-mixed-replace;boundary=" BOUNDARY "\r\n" \
            "\r\n" \
            "--" BOUNDARY "\r\n");

    if(write(fd, buffer, strlen(buffer)) < 0) {
        free(frame);
        return;
    }

    ROS_DEBUG("Headers send, sending stream now\n");

    while(!stop_requested_) {

        {
          /* wait for fresh frames */
          boost::unique_lock<boost::mutex> lock(image_buffer->mutex_);
          image_buffer->condition_.wait(lock);

          /* read buffer */
          frame_size = image_buffer->size_;

          /* check if framebuffer is large enough, increase it if necessary */
          if(frame_size > max_frame_size) {
              ROS_DEBUG("increasing buffer size to %d\n", frame_size);

              max_frame_size = frame_size + TEN_K;
              if((tmp = (unsigned char*)realloc(frame, max_frame_size)) == NULL) {
                  free(frame);
                  send_error(fd, 500, "not enough memory");
                  return;
              }

              frame = tmp;
          }

          /* copy v4l2_buffer timeval to user space */
          ros::Time time(image_buffer->time_stamp_);
          timestamp.tv_sec = time.sec;

          memcpy(frame, image_buffer->buffer_, frame_size);
          ROS_DEBUG("got frame (size: %d kB)\n", frame_size / 1024);
        }

        /*
         * print the individual mimetype and the length
         * sending the content-length fixes random stream disruption observed
         * with firefox
         */
        sprintf(buffer, "Content-Type: image/jpeg\r\n" \
                "Content-Length: %d\r\n" \
                "X-Timestamp: %d.%06d\r\n" \
                "\r\n", frame_size, (int)timestamp.tv_sec, (int)timestamp.tv_usec);
        ROS_DEBUG("sending intemdiate header\n");
        if(write(fd, buffer, strlen(buffer)) < 0) break;

        ROS_DEBUG("sending frame\n");
        if(write(fd, frame, frame_size) < 0) break;

        ROS_DEBUG("sending boundary\n");
        sprintf(buffer, "\r\n--" BOUNDARY "\r\n");
        if(write(fd, buffer, strlen(buffer)) < 0) break;
    }

    free(frame);
}

/******************************************************************************
Description.: Send HTTP header and copy the content of a file. To keep things
              simple, just a single folder gets searched for the file. Just
              files with known extension and supported mimetype get served.
              If no parameter was given, the file "index.html" will be copied.
Input Value.: * fd.......: filedescriptor to send data to
              * parameter: string that consists of the filename
              * id.......: specifies which server-context is the right one
Return Value: -
******************************************************************************/
void MJPEGServer::send_file(int fd, char *parameter)
{
    char buffer[BUFFER_SIZE] = {0};
    char *extension, *mimetype = NULL;
    int i, lfd;

    /* in case no parameter was given */
    if(parameter == NULL || strlen(parameter) == 0)
        parameter = "index.html";

    /* find file-extension */
    char * pch;
    pch = strchr(parameter, '.');
    int lastDot = 0;
    while(pch != NULL) {
        lastDot = pch - parameter;
        pch = strchr(pch + 1, '.');
    }

    if(lastDot == 0) {
        send_error(fd, 400, "No file extension found");
        return;
    } else {
        extension = parameter + lastDot;
        ROS_DEBUG("%s EXTENSION: %s\n", parameter, extension);
    }

    /* determine mime-type */
    for(i = 0; i < LENGTH_OF(mimetypes); i++) {
        if(strcmp(mimetypes[i].dot_extension, extension) == 0) {
            mimetype = (char *)mimetypes[i].mimetype;
            break;
        }
    }

    /* in case of unknown mimetype or extension leave */
    if(mimetype == NULL) {
        send_error(fd, 404, "MIME-TYPE not known");
        return;
    }

    /* now filename, mimetype and extension are known */
    ROS_DEBUG("trying to serve file \"%s\", extension: \"%s\" mime: \"%s\"\n", parameter, extension, mimetype);

    /* build the absolute path to the file */
    strncat(buffer, www_folder_, sizeof(buffer) - 1);
    strncat(buffer, parameter, sizeof(buffer) - strlen(buffer) - 1);

    /* try to open that file */
    if((lfd = open(buffer, O_RDONLY)) < 0) {
        ROS_DEBUG("file %s not accessible\n", buffer);
        send_error(fd, 404, "Could not open file");
        return;
    }
    ROS_DEBUG("opened file: %s\n", buffer);

    /* prepare HTTP header */
    sprintf(buffer, "HTTP/1.0 200 OK\r\n" \
            "Content-type: %s\r\n" \
            STD_HEADER \
            "\r\n", mimetype);
    i = strlen(buffer);

    /* first transmit HTTP-header, afterwards transmit content of file */
    do {
        if(write(fd, buffer, i) < 0) {
            close(lfd);
            return;
        }
    } while((i = read(lfd, buffer, sizeof(buffer))) > 0);

    /* close file, job done */
    close(lfd);
}

/******************************************************************************
Description.: Serve a connected TCP-client. This thread function is called
              for each connect of a HTTP client like a webbrowser. It determines
              if it is a valid HTTP request and dispatches between the different
              response options.
******************************************************************************/
void MJPEGServer::client(int fd) {
  int cnt;
  char input_suffixed = 0;
  char buffer[BUFFER_SIZE] = {0}, *pb = buffer;
  iobuffer iobuf;
  request req;

  /* initializes the structures */
  init_iobuffer(&iobuf);
  init_request(&req);

  /* What does the client want to receive? Read the request. */
  memset(buffer, 0, sizeof(buffer));
  if((cnt = _readline(fd, &iobuf, buffer, sizeof(buffer) - 1, 5)) == -1) {
      close(fd);
      return;
  }

  /* determine what to deliver */
  if(strstr(buffer, "GET /?action=stream") != NULL) {
    input_suffixed = 255;
    req.type = A_STREAM;
  } else if(strstr(buffer, "GET /?topic=") != NULL) {
    int len;
    input_suffixed = 255;
    req.type = A_TOPIC;

    /* advance by the length of known string */
    if((pb = strstr(buffer, "GET /?topic=")) == NULL) {
        ROS_DEBUG("HTTP request seems to be malformed\n");
        send_error(fd, 400, "Malformed HTTP request");
        close(fd);
        return;
    }
    pb += strlen("GET /?topic="); // a pb points to the string after the first & after command
    len = MIN(MAX(strspn(pb, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ._/-1234567890"), 0), 100);
    req.parameter = (char*)malloc(len + 1);
    if(req.parameter == NULL) {
        exit(EXIT_FAILURE);
    }
    memset(req.parameter, 0, len + 1);
    strncpy(req.parameter, pb, len);

    ROS_DEBUG("requested image topic: \"%s\"\n", len, req.parameter);
  } else {
    int len;

    ROS_DEBUG("try to serve a file\n");
    req.type = A_FILE;

    if((pb = strstr(buffer, "GET /")) == NULL) {
        ROS_DEBUG("HTTP request seems to be malformed\n");
        send_error(fd, 400, "Malformed HTTP request");
        close(fd);
        return;
    }

    pb += strlen("GET /");
    len = MIN(MAX(strspn(pb, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ._-1234567890"), 0), 100);
    req.parameter = (char*)malloc(len + 1);
    if(req.parameter == NULL) {
        exit(EXIT_FAILURE);
    }
    memset(req.parameter, 0, len + 1);
    strncpy(req.parameter, pb, len);

    ROS_DEBUG("parameter (len: %d): \"%s\"\n", len, req.parameter);
  }

  /*
   * parse the rest of the HTTP-request
   * the end of the request-header is marked by a single, empty line with "\r\n"
   */
  do {
    memset(buffer, 0, sizeof(buffer));

    if((cnt = _readline(fd, &iobuf, buffer, sizeof(buffer) - 1, 5)) == -1) {
        free_request(&req);
        close(fd);
        return;
    }

    if(strstr(buffer, "User-Agent: ") != NULL) {
        req.client = strdup(buffer + strlen("User-Agent: "));
    } else if(strstr(buffer, "Authorization: Basic ") != NULL) {
        req.credentials = strdup(buffer + strlen("Authorization: Basic "));
        decodeBase64(req.credentials);
        ROS_DEBUG("username:password: %s\n", req.credentials);
    }

  } while(cnt > 2 && !(buffer[0] == '\r' && buffer[1] == '\n'));



  /* now it's time to answer */
  switch(req.type) {
  case A_STREAM: {
      ROS_DEBUG("Request for stream\n");

      std::string topic = "/r_forearm_cam/image_color";

      // Subscribe to topic if not already done
      ImageSubscriberMap::iterator it = image_subscribers_.find(topic);
      if (it == image_subscribers_.end()) {
        image_subscribers_[topic] = image_transport_.subscribe(topic, 1, boost::bind(&MJPEGServer::imageCallback, this, _1, topic));
        image_buffers_[topic] = new ImageBuffer();
      }

      send_stream(fd, image_buffers_[topic]);
      break;
  }
  case A_TOPIC: {
      ROS_DEBUG("Request for topic\n");

      std::string topic = req.parameter;

      // Subscribe to topic if not already done
      ImageSubscriberMap::iterator it = image_subscribers_.find(topic);
      if (it == image_subscribers_.end()) {
        image_subscribers_[topic] = image_transport_.subscribe(topic, 1, boost::bind(&MJPEGServer::imageCallback, this, _1, topic));
        image_buffers_[topic] = new ImageBuffer();
      }

      send_stream(fd, image_buffers_[topic]);
      break;
  }
  case A_FILE: {
      if(www_folder_ == NULL)
          send_error(fd, 501, "no www-folder configured");
      else
          send_file(fd, req.parameter);
      break;
  }
  default:
      ROS_DEBUG("unknown request\n");
  }

  close(fd);
  free_request(&req);

  ROS_INFO("Disconnecting HTTP client\n");
  return;
}

void MJPEGServer::execute() {

  ROS_INFO("Starting mjpeg server");

  struct addrinfo *aip, *aip2;
  struct addrinfo hints;
  struct sockaddr_storage client_addr;
  socklen_t addr_len = sizeof(struct sockaddr_storage);
  fd_set selectfds;
  int max_fds = 0;

  char name[NI_MAXHOST];

  bzero(&hints, sizeof(hints));
  hints.ai_family = PF_UNSPEC;
  hints.ai_flags = AI_PASSIVE;
  hints.ai_socktype = SOCK_STREAM;

  int err;
  snprintf(name, sizeof(name), "%d", port_);
  if((err = getaddrinfo(NULL, name, &hints, &aip)) != 0) {
      perror(gai_strerror(err));
      exit(EXIT_FAILURE);
  }

  for(int i = 0; i < MAX_SD_LEN; i++)
      sd[i] = -1;


  /* open sockets for server (1 socket / address family) */
  int i = 0;
  int on;
  for(aip2 = aip; aip2 != NULL; aip2 = aip2->ai_next) {
      if((sd[i] = socket(aip2->ai_family, aip2->ai_socktype, 0)) < 0) {
          continue;
      }

      /* ignore "socket already in use" errors */
      on = 1;
      if(setsockopt(sd[i], SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
          perror("setsockopt(SO_REUSEADDR) failed");
      }

      /* IPv6 socket should listen to IPv6 only, otherwise we will get "socket already in use" */
      on = 1;
      if(aip2->ai_family == AF_INET6 && setsockopt(sd[i], IPPROTO_IPV6, IPV6_V6ONLY, (const void *)&on , sizeof(on)) < 0) {
          perror("setsockopt(IPV6_V6ONLY) failed");
      }

      /* perhaps we will use this keep-alive feature oneday */
      /* setsockopt(sd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on)); */

      if(bind(sd[i], aip2->ai_addr, aip2->ai_addrlen) < 0) {
          perror("bind");
          sd[i] = -1;
          continue;
      }

      if(listen(sd[i], 10) < 0) {
          perror("listen");
          sd[i] = -1;
      } else {
          i++;
          if(i >= MAX_SD_LEN) {
              ROS_ERROR("Maximum number of server sockets exceeded");
              i--;
              break;
          }
      }
  }

  sd_len = i;

  if(sd_len < 1) {
      ROS_ERROR("Bind(%d) failed", port_);
      closelog();
      exit(EXIT_FAILURE);
  }
  else {
      ROS_INFO("Bind(%d) succeeded", port_);
  }

  /* create a child for every client that connects */
  while(!stop_requested_) {

      ROS_INFO("waiting for clients to connect");

      do {
          FD_ZERO(&selectfds);

          for(i = 0; i < MAX_SD_LEN; i++) {
              if(sd[i] != -1) {
                  FD_SET(sd[i], &selectfds);

                  if(sd[i] > max_fds)
                      max_fds = sd[i];
              }
          }

          err = select(max_fds + 1, &selectfds, NULL, NULL, NULL);

          if(err < 0 && errno != EINTR) {
              perror("select");
              exit(EXIT_FAILURE);
          }
      } while(err <= 0);

      ROS_INFO("Client connected");

      for(i = 0; i < max_fds + 1; i++) {
          if(sd[i] != -1 && FD_ISSET(sd[i], &selectfds)) {
              int fd = accept(sd[i], (struct sockaddr *)&client_addr, &addr_len);

              /* start new thread that will handle this TCP connected client */
              ROS_DEBUG("create thread to handle client that just established a connection\n");

              if(getnameinfo((struct sockaddr *)&client_addr, addr_len, name, sizeof(name), NULL, 0, NI_NUMERICHOST) == 0) {
                  syslog(LOG_INFO, "serving client: %s\n", name);
              }

              boost::thread t(boost::bind( &MJPEGServer::client, this, fd ));
              t.detach();
          }
      }
  }

  ROS_INFO("leaving server thread, calling cleanup function now\n");
  cleanUp();
}

void MJPEGServer::cleanUp() {
  ROS_INFO("cleaning up ressources allocated by server thread");

  for(int i = 0; i < MAX_SD_LEN; i++)
      close(sd[i]);
}

void MJPEGServer::spin() {
  boost::thread t(boost::bind( &MJPEGServer::execute, this ));
  t.detach();
  ros::spin();
  ROS_INFO("stop requested");
  stop();
}

void MJPEGServer::stop() {
  stop_requested_ = true;
}

}

int main(int argc, char** argv){
  ros::init(argc, argv, "mjpeg_server");

  ros::NodeHandle nh;
  mjpeg_server::MJPEGServer server(nh);
  server.spin();

  return(0);
}

