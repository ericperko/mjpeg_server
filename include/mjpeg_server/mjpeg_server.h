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
#ifndef MJPEG_SERVER_H_
#define MJPEG_SERVER_H_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#define IO_BUFFER 256
#define BUFFER_SIZE 1024

/* the boundary is used for the M-JPEG stream, it separates the multipart stream of pictures */
#define BOUNDARY "boundarydonotcross"

/*
 * this defines the buffer size for a JPG-frame
 * selecting to large values will allocate much wasted RAM for each buffer
 * selecting to small values will lead to crashes due to to small buffers
 */
#define MAX_FRAME_SIZE (256*1024)
#define TEN_K (10*1024)

/*
 * Standard header to be send along with other header information like mimetype.
 *
 * The parameters should ensure the browser does not cache our answer.
 * A browser should connect for each file and not serve files from his cache.
 * Using cached pictures would lead to showing old/outdated pictures
 * Many browser seem to ignore, or at least not always obey those headers
 * since i observed caching of files from time to time.
 */
#define STD_HEADER "Connection: close\r\n" \
    "Server: MJPG-Streamer/0.2\r\n" \
    "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" \
    "Pragma: no-cache\r\n" \
    "Expires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"

/*
 * Maximum number of server sockets (i.e. protocol families) to listen.
 */
#define MAX_SD_LEN 50

namespace mjpeg_server {

/*
 * Only the following fileypes are supported.
 *
 * Other filetypes are simply ignored!
 * This table is a 1:1 mapping of files extension to a certain mimetype.
 */
static const struct {
    const char *dot_extension;
    const char *mimetype;
} mimetypes[] = {
    { ".html", "text/html" },
    { ".htm",  "text/html" },
    { ".css",  "text/css" },
    { ".js",   "text/javascript" },
    { ".txt",  "text/plain" },
    { ".jpg",  "image/jpeg" },
    { ".jpeg", "image/jpeg" },
    { ".png",  "image/png"},
    { ".gif",  "image/gif" },
    { ".ico",  "image/x-icon" },
    { ".swf",  "application/x-shockwave-flash" },
    { ".cab",  "application/x-shockwave-flash" },
    { ".jar",  "application/java-archive" },
    { ".json", "application/json" }
};

/* the webserver determines between these values for an answer */
typedef enum {
    A_UNKNOWN,
    A_SNAPSHOT,
    A_STREAM,
    A_COMMAND,
    A_FILE,
    A_INPUT_JSON,
    A_OUTPUT_JSON,
    A_PROGRAM_JSON,
} answer_t;

/*
 * the client sends information with each request
 * this structure is used to store the important parts
 */
typedef struct {
    answer_t type;
    char *parameter;
    char *client;
    char *credentials;
} request;

/* the iobuffer structure is used to read from the HTTP-client */
typedef struct {
    int level;              /* how full is the buffer */
    char buffer[IO_BUFFER]; /* the data */
} iobuffer;

/**
 * @class MJPEGServer
 * @brief
 */
class MJPEGServer {
public:
  /**
   * @brief  Constructor
   * @return
   */
  MJPEGServer();

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~MJPEGServer();

  /**
   * @brief  Runs whenever a new goal is sent to the move_base
   * @param goal The goal to pursue
   * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
   * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
   */
//  virtual robot_actions::ResultStatus execute(const ExploreGoal& goal, ExploreFeedback& feedback);
  void execute();

  // start server
  void spin();
  // stop server
  void stop();
  // closes client threads
  void cleanUp();
  // client thread
  void client(int fd);

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  void send_error(int fd, int which, char *message);
  void send_stream(int fd);
  void send_file(int fd, char *parameter);


  void init_iobuffer(iobuffer *iobuf);
  void init_request(request *req);
  void free_request(request *req);
  int _read(int fd, iobuffer *iobuf, void *buffer, size_t len, int timeout);
  int _readline(int fd, iobuffer *iobuf, char *buffer, size_t len, int timeout);
  void decodeBase64(char *data);
  int hex_char_to_int(char in);
  int unescape(char *string);


  ros::NodeHandle node_;

  boost::mutex client_mutex_;
  int port_;

  int sd[MAX_SD_LEN];
  int sd_len;

  bool stop_requested_;
  char* credentials_;
  char* www_folder_;

  boost::condition_variable image_process_condition;
  boost::mutex image_process_mutex;

  int image_size_;
  double image_time_stamp_;
  char* image_buffer_;


};

}

#endif

