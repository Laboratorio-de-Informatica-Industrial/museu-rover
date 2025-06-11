import sys
import rospy
from PyQt5.QtWidgets import (
    QApplication, QLabel, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QMessageBox, QFrame, QSizePolicy
)
from PyQt5.QtGui import QPixmap, QImage, QFont, QColor
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber(QThread):
    image_signal = pyqtSignal(QImage)
    connection_status_signal = pyqtSignal(str) # Signal for connection status

    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.subscriber = None
        self.running = True
        self.last_message_time = rospy.get_time() # To track last message arrival
        self.connection_check_timer = QTimer()
        self.connection_check_timer.setInterval(1000) # Check every 1 second
        self.connection_check_timer.timeout.connect(self._check_connection)
        self.connection_check_timer.start()

        rospy.loginfo(f"ImageSubscriber initialized for topic: {self.topic_name}")

    def run(self):
        try:
            self.subscriber = rospy.Subscriber(self.topic_name, CompressedImage, self.image_callback, queue_size=1)
            rospy.loginfo(f"Subscribing to {self.topic_name} with message type CompressedImage.")
            # rospy.spin() needs to be in the thread that created the subscriber
            # Instead of rospy.spin(), we'll just keep the thread alive
            # and let the subscriber callbacks happen.
            while self.running and not rospy.is_shutdown():
                rospy.rostime.wallsleep(0.1) # Small sleep to not busy-wait
            rospy.loginfo("ImageSubscriber thread stopping.")
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt received, subscriber thread shutting down.")
        except Exception as e:
            rospy.logerr(f"Error in ImageSubscriber run: {e}")
        finally:
            self.connection_check_timer.stop() # Stop the timer when thread finishes


    def image_callback(self, data):
        self.last_message_time = rospy.get_time() # Update last message time
        if not self.running:
            return

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            if cv_image.size == 0:
                # rospy.logwarn("Converted OpenCV image is empty!") # Avoid excessive logging
                return
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(rgb_image.copy().data, width, height, bytes_per_line, QImage.Format_RGB888)

        if q_image.isNull():
            rospy.logerr("QImage conversion resulted in a null image!")
            return

        self.image_signal.emit(q_image)

    def _check_connection(self):
        # Check if we've received a message in the last 2 seconds
        if (rospy.get_time() - self.last_message_time) > 2.0:
            self.connection_status_signal.emit("Status: Disconnected / No Stream")
        else:
            self.connection_status_signal.emit("Status: Connected / Streaming")

    def stop(self):
        self.running = False
        self.connection_check_timer.stop()
        if self.subscriber:
            rospy.loginfo("Unregistering ROS subscriber...")
            self.subscriber.unregister()
            self.subscriber = None
        rospy.loginfo("ImageSubscriber stop method completed.")


class ImageViewer(QWidget):
    def __init__(self, topic_name):
        super().__init__()
        self.setWindowTitle("Leo Rover")
        self.setFixedSize(1024, 728) # Set a fixed window size for better control

        # --- Main Layout ---
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0) # Add some padding
        main_layout.setSpacing(0) # Space between widgets

        # --- Header Section (Title) ---
        header_layout = QHBoxLayout()
        self.title_label = QLabel("Museum Remote Visit - Live Feed")
        self.title_label.setFont(QFont("Arial", 24, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("color: #333333;") # Dark gray text
        header_layout.addWidget(self.title_label)
        main_layout.addLayout(header_layout)

        # --- Video Frame ---
        video_frame = QFrame()
        video_frame.setFrameShape(QFrame.Box)
        video_frame.setFrameShadow(QFrame.Raised)
        video_frame.setLineWidth(2)
        video_frame.setStyleSheet("border: 0px solid #555555; background-color: black;") # Dark border, black background for video
        video_layout = QVBoxLayout(video_frame)
        video_layout.setContentsMargins(5, 5, 5, 5) # Padding inside video frame

        self.image_label = QLabel("Waiting for video stream...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(1024, 768) # Fixed size for video feed
        self.image_label.setStyleSheet("color: white;") # Text color for 'Waiting for stream'
        video_layout.addWidget(self.image_label)
        main_layout.addWidget(video_frame)
    
        # --- Status Bar ---
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Status: Initializing...")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setStyleSheet("color: #006600; font-weight: bold;") # Green text for status
        status_layout.addWidget(self.status_label)
        status_layout.addStretch(1) # Push status to the left
        main_layout.addLayout(status_layout)

        self.setLayout(main_layout)

        self.image_subscriber = ImageSubscriber(topic_name)
        self.image_subscriber.image_signal.connect(self.update_image)
        self.image_subscriber.connection_status_signal.connect(self.update_status)
        self.image_subscriber.start()
        rospy.loginfo("ImageViewer initialized and ImageSubscriber thread started.")


    def update_image(self, q_image):
        if not q_image.isNull():
            # Scale the QImage to fit the QLabel while maintaining aspect ratio
            # Use background color for QLabel if image is smaller, or scale to fill
            pixmap = QPixmap.fromImage(q_image).scaled(
                self.image_label.width(), self.image_label.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.image_label.setPixmap(pixmap)
            self.image_label.setText("") # Clear "Waiting for video stream..."
        else:
            self.image_label.setText("Error: No valid image received.")
            self.image_label.setPixmap(QPixmap()) # Clear any old pixmap

    def update_status(self, status_message):
        self.status_label.setText(status_message)
        if "Disconnected" in status_message or "No Stream" in status_message:
            self.status_label.setStyleSheet("color: #CC0000; font-weight: bold;") # Red for disconnected
        else:
            self.status_label.setStyleSheet("color: #006600; font-weight: bold;") # Green for connected

    def closeEvent(self, event):
        rospy.loginfo("Closing ImageViewer, stopping ROS thread...")
        self.image_subscriber.stop()
        self.image_subscriber.wait(2000)
        if self.image_subscriber.isRunning():
            rospy.logwarn("Image subscriber thread did not terminate cleanly. Forcing quit.")
            self.image_subscriber.terminate()
            self.image_subscriber.wait()

        if not rospy.is_shutdown():
            rospy.signal_shutdown("PyQt5 application closing")
            rospy.loginfo("rospy.signal_shutdown called.")

        event.accept()

# Custom message box replacement for alert/confirm
def show_message_box(title, message, icon=QMessageBox.Information):
    msg_box = QMessageBox()
    msg_box.setIcon(icon)
    msg_box.setWindowTitle(title)
    msg_box.setText(message)
    msg_box.setStandardButtons(QMessageBox.Ok)
    msg_box.exec_()


if __name__ == '__main__':
    try:
        # Use a more descriptive node name for the museum app
        rospy.init_node('museum_remote_visit_transmitter', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo("ROS node 'museum_remote_visit_transmitter' initialized in main thread.")
    except rospy.exceptions.ROSException as e:
        show_message_box("ROS Initialization Error",
                         f"Error initializing ROS node: {e}\n\nPlease ensure ROS is running and configured correctly.",
                         QMessageBox.Critical)
        sys.exit(1)

    # IMPORTANT: Replace with your Leo Rover's actual camera topic
    # Ensure this topic is actually publishing CompressedImage messages
    CAMERA_TOPIC = "/leo_main_duque/camera/image_raw/compressed"

    app = QApplication(sys.argv)
    viewer = ImageViewer(CAMERA_TOPIC)
    viewer.show()
    sys.exit(app.exec_())