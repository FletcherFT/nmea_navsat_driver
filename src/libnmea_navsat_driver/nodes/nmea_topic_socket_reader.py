# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_topic_serial_reader executable."""

import select
import sys
import traceback

try:
    import socketserver
except ImportError:
    import SocketServer as socketserver  # Python 2.7

import rospy

from nmea_msgs.msg import Sentence
from libnmea_navsat_driver.driver import RosNMEADriver

class NMEAMessageHandler(socketserver.DatagramRequestHandler):
    def handle(self):
        for line in self.rfile:
            line = line.strip()
            if not line:
                continue

            try:
                sentence = Sentence()
                sentence.header.stamp = rospy.get_rostime()
                sentence.header.frame_id = self.server.frame_id
                sentence.sentence = line.decode('ascii')
                self.server.publisher.publish(sentence)
            except UnicodeError as e:
                rospy.logwarn("Skipped reading a line from the UDP socket because it could not be "
                              "decoded as an ASCII string. The bytes were {0}".format(line))
            except ValueError:
                rospy.logwarn(
                    "ValueError, likely due to missing fields in the NMEA "
                    "message. Please report this issue at "
                    "https://github.com/ros-drivers/nmea_navsat_driver"
                    ", including the following:\n\n"
                    "```\n" +
                    repr(line) + "\n\n" +
                    traceback.format_exc() +
                    "```")


def main():
    """Create and run the nmea_topic_socket_reader ROS node.

    Opens a UDP port and publishes data from the port as nmea_msgs.msg.Sentence messages.

    :ROS Parameters:
        - ~ip (str)
            IPV4 address of the socket to open.
        - ~port (int)
            Local port of the socket to open.
        - ~timeout (float)
            The time out period for the socket, in seconds.
    :ROS Publishers:
        - nmea_sentence (nmea_msgs.msg.Sentence)
            Publishes each line from the UDP port as a new message. The header's stamp is
            set to the rostime when the data is received from the port.
    """
    rospy.init_node('nmea_topic_socket_reader')
    try:
        local_ip = rospy.get_param('~ip', '0.0.0.0')
        local_port = rospy.get_param('~port', 10110)
        timeout = rospy.get_param('~timeout_sec', 2)
    except KeyError as e:
        rospy.logerr("Parameter %s not found" % e)
        sys.exit(1)
    nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size=1)

    # Create a socket
    server = socketserver.UDPServer((local_ip, local_port), NMEAMessageHandler,
                                    bind_and_activate=False)
    server.frame_id = RosNMEADriver.get_frame_id()
    server.publisher = nmea_pub

    # Start listening for connections
    server.server_bind()
    server.server_activate()

    # Handle incoming connections until ROS shuts down
    try:
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([server], [], [], timeout)
            if server in rlist:
                server.handle_request()
    except rospy.ROSInterruptException:
        pass
    except Exception:
        rospy.logerr(traceback.format_exc())
    finally:
        server.server_close()
