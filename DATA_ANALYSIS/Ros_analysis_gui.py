import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QComboBox, QCheckBox, QLabel, QPushButton, 
                            QSlider, QGroupBox, QFileDialog, QListWidget, QSplitter,
                            QMessageBox, QColorDialog, QListWidgetItem, QTabWidget)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QColor, QIcon, QPixmap, QImage
from bagpy import bagreader
from scipy.signal import butter, sosfiltfilt
import random
import os
import cv2
import rosbag
from PIL import Image
import struct

class ImageDisplayWidget(QWidget):
    """Widget to display camera frames from ROS bag files"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        
        # Create a label to display the image
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setText("No image loaded")
        self.layout.addWidget(self.image_label)
        
        # Create controls for navigating frames
        controls_layout = QHBoxLayout()
        
        self.prev_btn = QPushButton("Previous Frame")
        self.prev_btn.clicked.connect(self.prev_frame)
        controls_layout.addWidget(self.prev_btn)
        
        self.frame_slider = QSlider(Qt.Horizontal)
        self.frame_slider.setEnabled(False)
        self.frame_slider.valueChanged.connect(self.slider_changed)
        controls_layout.addWidget(self.frame_slider)
        
        self.frame_label = QLabel("Frame: 0/0")
        controls_layout.addWidget(self.frame_label)
        
        self.next_btn = QPushButton("Next Frame")
        self.next_btn.clicked.connect(self.next_frame)
        controls_layout.addWidget(self.next_btn)
        
        self.layout.addLayout(controls_layout)
        
        # Add a checkbox to limit frames
        limit_layout = QHBoxLayout()
        self.limit_frames_cb = QCheckBox("Limit to 100 frames (faster loading)")
        self.limit_frames_cb.setChecked(True)
        limit_layout.addWidget(self.limit_frames_cb)
        self.layout.addLayout(limit_layout)
        
        # Initialize variables
        self.frames = []
        self.current_frame = 0
        self.bag_file = None
        self.topic = None
        self.total_frames = 0
        self.frame_timestamps = []
        self.preloaded_frames = {}  # Cache for preloaded frames
        self.preload_buffer_size = 10  # Number of frames to keep in memory
        
    def load_frames(self, bag_file, topic):
        """Load frames from the specified topic in the bag file"""
        try:
            self.statusBar().showMessage(f"Loading frames from {topic}...")
        except:
            print(f"Loading frames from {topic}...")
            
        try:
            # Clear existing frames
            self.frames = []
            self.preloaded_frames = {}
            self.frame_timestamps = []
            self.current_frame = 0
            
            # Store bag file and topic for later use
            self.bag_file = bag_file
            self.topic = topic
            
            # Count total frames and store timestamps
            bag = rosbag.Bag(bag_file)
            frame_count = 0
            
            for _, msg, t in bag.read_messages(topics=[topic]):
                self.frame_timestamps.append((frame_count, t.to_sec(), msg))
                frame_count += 1
                
                # If limiting frames, stop after 100
                if self.limit_frames_cb.isChecked() and frame_count >= 100:
                    break
                    
                # Update progress every 100 frames
                if frame_count % 100 == 0:
                    try:
                        self.statusBar().showMessage(f"Counting frames: {frame_count} found so far...")
                        QApplication.processEvents()
                    except:
                        print(f"Counting frames: {frame_count} found so far...")
            
            bag.close()
            
            self.total_frames = frame_count
            
            if self.total_frames == 0:
                print("No frames found in the topic")
                self.frames = [self._create_dummy_frame(f"No frames found in {topic}", 0)]
                self.current_frame = 0
                self.update_display()
                return False
                
            print(f"Found {self.total_frames} frames in topic {topic}")
            
            # Set up slider
            self.frame_slider.setEnabled(True)
            self.frame_slider.setMinimum(0)
            self.frame_slider.setMaximum(self.total_frames - 1)
            self.frame_slider.setValue(0)
            
            # Preload first frame
            self._load_frame(0)
            self.update_display()
            
            return True
            
        except Exception as e:
            print(f"Error loading frames: {str(e)}")
            import traceback
            traceback.print_exc()
            
            # Create dummy frames to show error
            self.frames = [self._create_dummy_frame(f"Error: {str(e)}", 0)]
            if self.frames:
                self.current_frame = 0
                self.update_display()
            
            return False
    
    def _load_frame(self, index):
        """Load a specific frame by index"""
        if index < 0 or index >= self.total_frames:
            return False
            
        # Check if frame is already preloaded
        if index in self.preloaded_frames:
            return True
            
        try:
            # Get the message from stored timestamps
            _, _, msg = self.frame_timestamps[index]
            
            # Convert message to image
            pixmap = self._convert_msg_to_pixmap(msg)
            if pixmap:
                self.preloaded_frames[index] = pixmap
                
                # Remove old frames from cache if it gets too large
                if len(self.preloaded_frames) > self.preload_buffer_size:
                    # Keep current frame and neighbors, remove the oldest ones
                    keep_indices = list(range(max(0, index - 5), min(self.total_frames, index + 6)))
                    keys_to_remove = [k for k in self.preloaded_frames.keys() 
                                     if k not in keep_indices and k != index]
                    
                    # Sort by distance from current index and remove furthest
                    keys_to_remove.sort(key=lambda k: abs(k - index), reverse=True)
                    
                    # Remove excess frames
                    excess = len(self.preloaded_frames) - self.preload_buffer_size
                    for k in keys_to_remove[:excess]:
                        del self.preloaded_frames[k]
                
                return True
        except Exception as e:
            print(f"Error loading frame {index}: {str(e)}")
            
        return False
    
    def _convert_msg_to_pixmap(self, msg):
        """Convert a ROS image message to QPixmap"""
        try:
            # Try different methods to convert the message to an image
            cv_image = None
            
            if hasattr(msg, 'format') and msg.format == 'rgb8':
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = img_data.reshape((msg.height, msg.width, 3))
            elif hasattr(msg, 'format') and msg.format == 'bgr8':
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = img_data.reshape((msg.height, msg.width, 3))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif hasattr(msg, 'encoding'):
                if msg.encoding == 'rgb8':
                    img_data = np.frombuffer(msg.data, dtype=np.uint8)
                    cv_image = img_data.reshape((msg.height, msg.width, 3))
                elif msg.encoding == 'bgr8':
                    img_data = np.frombuffer(msg.data, dtype=np.uint8)
                    cv_image = img_data.reshape((msg.height, msg.width, 3))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                elif msg.encoding == 'mono8':
                    img_data = np.frombuffer(msg.data, dtype=np.uint8)
                    cv_image = img_data.reshape((msg.height, msg.width))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
                else:
                    # Try a generic approach
                    img_data = np.frombuffer(msg.data, dtype=np.uint8)
                    if msg.step == msg.width:  # Mono
                        cv_image = img_data.reshape((msg.height, msg.width))
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
                    else:  # Assume 3 channels
                        channels = msg.step // msg.width
                        cv_image = img_data.reshape((msg.height, msg.width, channels))
                        if channels == 3:
                            # Assume BGR and convert to RGB
                            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                # Try compressed image formats
                if hasattr(msg, 'format') and msg.format.lower() == 'jpeg':
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                elif hasattr(msg, 'format') and msg.format.lower() == 'png':
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else:
                    # Last resort - try to interpret as raw data
                    if hasattr(msg, 'width') and hasattr(msg, 'height'):
                        width = msg.width
                        height = msg.height
                        # Guess channels based on data length
                        data_len = len(msg.data)
                        if data_len == width * height:  # Mono
                            channels = 1
                        elif data_len == width * height * 3:  # RGB/BGR
                            channels = 3
                        else:
                            raise ValueError(f"Can't determine image format from data length {data_len}")
                        
                        img_data = np.frombuffer(msg.data, dtype=np.uint8)
                        if channels == 1:
                            cv_image = img_data.reshape((height, width))
                            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
                        else:
                            cv_image = img_data.reshape((height, width, channels))
                            # Assume BGR and convert to RGB
                            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                    else:
                        raise ValueError("Message doesn't have width/height attributes")
            
            if cv_image is None:
                raise ValueError("Failed to convert message to image")
                
            # Convert to QPixmap
            if len(cv_image.shape) == 2:  # Grayscale
                height, width = cv_image.shape
                bytes_per_line = width
                q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
            else:  # RGB
                height, width, channels = cv_image.shape
                bytes_per_line = channels * width
                q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            return QPixmap.fromImage(q_image)
            
        except Exception as e:
            print(f"Error converting message to pixmap: {str(e)}")
            return None
    
    def _create_dummy_frame(self, text, index):
        """Create a dummy frame with text for testing"""
        # Create a blank image
        image = QPixmap(640, 480)
        image.fill(QColor(40, 40, 40))
        
        # Add text
        from PyQt5.QtGui import QPainter, QFont, QPen
        painter = QPainter(image)
        painter.setPen(QPen(QColor(255, 255, 255)))
        font = QFont()
        font.setPointSize(20)
        painter.setFont(font)
        painter.drawText(image.rect(), Qt.AlignCenter, f"{text}\nFrame {index+1}")
        painter.end()
        
        return image
    
    def update_display(self):
        """Update the display with the current frame"""
        if self.total_frames == 0:
            self.image_label.setText("No frames loaded")
            self.frame_label.setText("Frame: 0/0")
            return
            
        # Make sure current frame is loaded
        self._load_frame(self.current_frame)
        
        # Also preload next and previous frames
        if self.current_frame > 0:
            self._load_frame(self.current_frame - 1)
        if self.current_frame < self.total_frames - 1:
            self._load_frame(self.current_frame + 1)
            
        # Display current frame
        if self.current_frame in self.preloaded_frames:
            self.image_label.setPixmap(self.preloaded_frames[self.current_frame])
            self.frame_label.setText(f"Frame: {self.current_frame+1}/{self.total_frames}")
            
            # Update slider without triggering valueChanged
            self.frame_slider.blockSignals(True)
            self.frame_slider.setValue(self.current_frame)
            self.frame_slider.blockSignals(False)
        else:
            self.image_label.setText(f"Error loading frame {self.current_frame+1}")
        
    def next_frame(self):
        """Show the next frame"""
        if self.total_frames > 0 and self.current_frame < self.total_frames - 1:
            self.current_frame += 1
            self.update_display()
            
    def prev_frame(self):
        """Show the previous frame"""
        if self.total_frames > 0 and self.current_frame > 0:
            self.current_frame -= 1
            self.update_display()
            
    def slider_changed(self, value):
        """Handle slider value change"""
        if self.total_frames > 0 and 0 <= value < self.total_frames:
            self.current_frame = value
            self.update_display()

class ColorBox(QWidget):
    """Small colored box widget to display a topic's color"""
    def __init__(self, color, parent=None):
        super().__init__(parent)
        self.color = color
        self.setMinimumSize(16, 16)
        self.setMaximumSize(16, 16)
        
    def paintEvent(self, event):
        painter = self.style().drawPrimitive
        pixmap = QPixmap(16, 16)
        pixmap.fill(self.color)
        self.setIcon(QIcon(pixmap))

class TopicItem:
    """Class to store topic information"""
    def __init__(self, name, color=None):
        self.name = name
        # Generate a random color if none provided
        if color is None:
            r = random.randint(50, 200)
            g = random.randint(50, 200)
            b = random.randint(50, 200)
            self.color = QColor(r, g, b)
        else:
            self.color = color
        self.show_raw = True
        self.show_filtered = True

class BagVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Topic ROS Bag Visualizer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize variables
        self.bag_file = None
        self.bag_reader = None
        self.available_topics = []
        self.selected_topics = []  # List of TopicItem objects
        self.topic_data = {}
        self.cutoff = 0.1
        self.filter_order = 2
        self.topic_colors = {}  # Store colors for topics
        self.camera_topics = []  # Store camera topics
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Create top control panel
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        
        # File selection button
        self.file_btn = QPushButton("Select Bag File")
        self.file_btn.clicked.connect(self.select_bag_file)
        control_layout.addWidget(self.file_btn)
        
        # File info label
        self.file_label = QLabel("No file selected")
        control_layout.addWidget(self.file_label)
        
        # Add control panel to main layout
        main_layout.addWidget(control_panel)
        
        # Create tab widget for plot and video views
        self.tab_widget = QTabWidget()
        
        # Add tab for plot view
        self.plot_tab = QWidget()
        self.tab_widget.addTab(self.plot_tab, "Plot View")
        
        # Add tab for video preview
        self.video_tab = QWidget()
        self.tab_widget.addTab(self.video_tab, "Video Preview")
        
        # Set up video tab
        video_tab_layout = QVBoxLayout(self.video_tab)
        self.image_display = ImageDisplayWidget()
        video_tab_layout.addWidget(self.image_display)
        
        # Set up plot tab
        plot_tab_layout = QVBoxLayout(self.plot_tab)
        
        # Create splitter for resizable panels in plot tab
        plot_splitter = QSplitter(Qt.Horizontal)
        
        # Left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(5, 5, 5, 5)
        
        # Topic selection group
        topic_group = QGroupBox("Topic Selection")
        topic_layout = QVBoxLayout(topic_group)
        
        # Available topics list with label
        topic_layout.addWidget(QLabel("Available Topics:"))
        self.topic_combo = QComboBox()
        topic_layout.addWidget(self.topic_combo)
        
        # Add button
        self.add_topic_btn = QPushButton("Add Topic")
        self.add_topic_btn.clicked.connect(self.add_selected_topic)
        topic_layout.addWidget(self.add_topic_btn)
        
        # Selected topics list with label
        topic_layout.addWidget(QLabel("Selected Topics:"))
        self.selected_topics_list = QListWidget()
        self.selected_topics_list.setSelectionMode(QListWidget.SingleSelection)
        self.selected_topics_list.itemClicked.connect(self.topic_item_clicked)
        topic_layout.addWidget(self.selected_topics_list)
        
        # Topic control buttons
        topic_buttons_layout = QHBoxLayout()
        
        self.remove_topic_btn = QPushButton("Remove Topic")
        self.remove_topic_btn.clicked.connect(self.remove_selected_topic)
        topic_buttons_layout.addWidget(self.remove_topic_btn)
        
        self.change_color_btn = QPushButton("Change Color")
        self.change_color_btn.clicked.connect(self.change_topic_color)
        topic_buttons_layout.addWidget(self.change_color_btn)
        
        topic_layout.addLayout(topic_buttons_layout)
        
        # Display options for selected topic
        self.topic_options_group = QGroupBox("Topic Display Options")
        topic_options_layout = QVBoxLayout(self.topic_options_group)
        
        self.show_raw_cb = QCheckBox("Show Raw Data")
        self.show_raw_cb.setChecked(True)
        self.show_raw_cb.stateChanged.connect(self.update_topic_display_options)
        topic_options_layout.addWidget(self.show_raw_cb)
        
        self.show_filtered_cb = QCheckBox("Show Filtered Data")
        self.show_filtered_cb.setChecked(True)
        self.show_filtered_cb.stateChanged.connect(self.update_topic_display_options)
        topic_options_layout.addWidget(self.show_filtered_cb)
        
        # Add groups to left panel
        left_layout.addWidget(topic_group)
        left_layout.addWidget(self.topic_options_group)
        
        # Filter controls
        filter_group = QGroupBox("Filter Controls")
        filter_layout = QVBoxLayout(filter_group)
        
        # Cutoff frequency slider
        cutoff_layout = QHBoxLayout()
        cutoff_layout.addWidget(QLabel("Cutoff Frequency:"))
        self.cutoff_label = QLabel(f"{self.cutoff:.2f} Hz")
        cutoff_layout.addWidget(self.cutoff_label)
        filter_layout.addLayout(cutoff_layout)
        
        self.cutoff_slider = QSlider(Qt.Horizontal)
        self.cutoff_slider.setRange(1, 100)  # 0.01 to 1.0
        self.cutoff_slider.setValue(int(self.cutoff * 100))
        self.cutoff_slider.valueChanged.connect(self.update_cutoff)
        filter_layout.addWidget(self.cutoff_slider)
        
        # Filter order slider
        order_layout = QHBoxLayout()
        order_layout.addWidget(QLabel("Filter Order:"))
        self.order_label = QLabel(f"{self.filter_order}")
        order_layout.addWidget(self.order_label)
        filter_layout.addLayout(order_layout)
        
        self.order_slider = QSlider(Qt.Horizontal)
        self.order_slider.setRange(1, 6)  # Reduced maximum order
        self.order_slider.setValue(self.filter_order)
        self.order_slider.valueChanged.connect(self.update_order)
        filter_layout.addWidget(self.order_slider)
        
        # Add filter group
        left_layout.addWidget(filter_group)
        
        # Add a refresh button
        self.refresh_btn = QPushButton("Refresh Plot")
        self.refresh_btn.clicked.connect(self.update_plot)
        left_layout.addWidget(self.refresh_btn)
        
        # Add camera topic selection
        camera_group = QGroupBox("Camera Preview")
        camera_layout = QVBoxLayout(camera_group)
        
        camera_layout.addWidget(QLabel("Available Camera Topics:"))
        self.camera_combo = QComboBox()
        camera_layout.addWidget(self.camera_combo)
        
        self.load_video_btn = QPushButton("Load Camera Frames")
        self.load_video_btn.clicked.connect(self.load_camera_frames)
        camera_layout.addWidget(self.load_video_btn)
        
        # Add new button for rosbag_to_video conversion
        self.convert_video_btn = QPushButton("Convert to Video File")
        self.convert_video_btn.clicked.connect(self.convert_to_video)
        camera_layout.addWidget(self.convert_video_btn)
        
        left_layout.addWidget(camera_group)
        
        # Add some stretch to push everything up
        left_layout.addStretch()
        
        # Right panel for matplotlib plot
        self.plot_widget = QWidget()
        plot_layout = QVBoxLayout(self.plot_widget)
        plot_layout.setContentsMargins(5, 5, 5, 5)
        
        # Create matplotlib figure and canvas
        self.figure = plt.figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)
        
        # Add panels to splitter
        plot_splitter.addWidget(left_panel)
        plot_splitter.addWidget(self.plot_widget)
        
        # Set initial sizes
        plot_splitter.setSizes([300, 900])
        
        # Add splitter to plot tab layout
        plot_tab_layout.addWidget(plot_splitter)
        
        # Add tab widget to main layout
        main_layout.addWidget(self.tab_widget)
        
        # Status bar setup
        self.statusBar().showMessage("Ready")
    
    def select_bag_file(self):
        """Open file dialog to select a bag file"""
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(self, "Select Bag File", "", "Bag Files (*.bag)")
        
        if file_path:
            self.statusBar().showMessage(f"Loading bag file: {file_path}")
            self.file_label.setText(f"File: {file_path.split('/')[-1]}")
            self.load_bag_file(file_path)
    
    def load_bag_file(self, file_path):
        """Load the selected bag file and populate topic dropdowns"""
        try:
            self.bag_file = file_path
            self.bag_reader = bagreader(file_path)
            
            # Clear existing data
            self.topic_data = {}
            self.selected_topics = []
            self.selected_topics_list.clear()
            self.camera_topics = []
            self.camera_combo.clear()
            
            # Get available topics
            topic_table = self.bag_reader.topic_table
            self.available_topics = list(topic_table['Topics'])
            
            # Update topic selection combobox
            self.topic_combo.clear()
            for topic in self.available_topics:
                self.topic_combo.addItem(topic)
                
                # Identify camera topics
                if 'usb_cam' in topic or 'image' in topic or 'camera' in topic:
                    self.camera_topics.append(topic)
                    self.camera_combo.addItem(topic)
            
            self.statusBar().showMessage("Bag file loaded successfully")
            
            # If camera topics were found, switch to video tab and auto-load first camera
            if self.camera_topics:
                self.statusBar().showMessage(f"Found {len(self.camera_topics)} camera topics")
                self.tab_widget.setCurrentIndex(1)  # Switch to video tab
                # Auto-load the first camera topic
                if self.camera_combo.count() > 0:
                    self.camera_combo.setCurrentIndex(0)
                    self.load_camera_frames()
            
            self.update_plot()
            
        except Exception as e:
            self.statusBar().showMessage(f"Error loading bag file: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to load bag file: {str(e)}")
    
    def add_selected_topic(self):
        """Add the currently selected topic from combobox to the list of plotted topics"""
        topic_name = self.topic_combo.currentText()
        if not topic_name or any(t.name == topic_name for t in self.selected_topics):
            return
        
        # Create a new topic item
        topic_item = TopicItem(topic_name)
        self.selected_topics.append(topic_item)
        
        # Create list widget item with color indicator
        list_item = QListWidgetItem(topic_name)
        pixmap = QPixmap(16, 16)
        pixmap.fill(topic_item.color)
        list_item.setIcon(QIcon(pixmap))
        
        self.selected_topics_list.addItem(list_item)
        self.update_plot()
    
    def remove_selected_topic(self):
        """Remove the selected topic from the list"""
        current_row = self.selected_topics_list.currentRow()
        if current_row >= 0:
            removed_topic = self.selected_topics.pop(current_row)
            self.selected_topics_list.takeItem(current_row)
            self.update_plot()
    
    def change_topic_color(self):
        """Change the color of the selected topic"""
        current_row = self.selected_topics_list.currentRow()
        if current_row >= 0:
            topic_item = self.selected_topics[current_row]
            color_dialog = QColorDialog()
            new_color = color_dialog.getColor(initial=topic_item.color)
            
            if new_color.isValid():
                topic_item.color = new_color
                
                # Update icon in list widget
                list_item = self.selected_topics_list.item(current_row)
                pixmap = QPixmap(16, 16)
                pixmap.fill(new_color)
                list_item.setIcon(QIcon(pixmap))
                
                self.update_plot()
    
    def topic_item_clicked(self, item):
        """Handle click on a topic in the list"""
        current_row = self.selected_topics_list.currentRow()
        if current_row >= 0:
            topic_item = self.selected_topics[current_row]
            
            # Update checkboxes to reflect the selected topic's settings
            self.show_raw_cb.setChecked(topic_item.show_raw)
            self.show_filtered_cb.setChecked(topic_item.show_filtered)
    
    def update_topic_display_options(self):
        """Update display options for the selected topic"""
        current_row = self.selected_topics_list.currentRow()
        if current_row >= 0:
            topic_item = self.selected_topics[current_row]
            topic_item.show_raw = self.show_raw_cb.isChecked()
            topic_item.show_filtered = self.show_filtered_cb.isChecked()
            self.update_plot()
    
    def update_cutoff(self):
        """Update cutoff frequency from slider"""
        self.cutoff = self.cutoff_slider.value() / 100.0
        self.cutoff_label.setText(f"{self.cutoff:.2f} Hz")
        self.update_plot()
    
    def update_order(self):
        """Update filter order from slider"""
        self.filter_order = self.order_slider.value()
        self.order_label.setText(f"{self.filter_order}")
        self.update_plot()
    
    def get_topic_data(self, topic):
        """Load topic data if not already loaded"""
        if topic not in self.topic_data and self.bag_reader is not None:
            try:
                self.statusBar().showMessage(f"Loading data for topic: {topic}")
                df = pd.read_csv(self.bag_reader.message_by_topic(topic))
                self.topic_data[topic] = df
                self.statusBar().showMessage(f"Data loaded for {topic}")
            except Exception as e:
                self.statusBar().showMessage(f"Error loading topic data: {str(e)}")
                QMessageBox.warning(self, "Warning", f"Failed to load data for topic {topic}: {str(e)}")
                return None
        
        return self.topic_data.get(topic, None)
    
    def safe_filter(self, data, cutoff, fs, order=2):
        """Safely apply filter based on data length"""
        # Check if we have enough data points for the requested filter order
        if len(data) < 3 * order:
            # Fall back to a simple moving average if not enough data
            window_size = min(5, len(data) // 2) if len(data) > 10 else 3
            return data.rolling(window=window_size, center=True).mean().fillna(method='ffill').fillna(method='bfill')
        
        try:
            # Try to use sosfiltfilt (better phase response and numerical stability)
            nyquist = 0.5 * fs
            normal_cutoff = cutoff / nyquist
            
            # Use second-order sections (sos) format which is more numerically stable
            sos = butter(order, normal_cutoff, btype='low', analog=False, output='sos')
            filtered = sosfiltfilt(sos, data)
            return filtered
        except Exception as e:
            # If filtfilt fails, try a simpler approach
            self.statusBar().showMessage(f"Filter warning: {str(e)}. Using simple filter.")
            # Simple moving average as fallback
            window_size = min(5, len(data) // 2) if len(data) > 10 else 3
            return data.rolling(window=window_size, center=True).mean().fillna(method='ffill').fillna(method='bfill')
    
    def find_data_column(self, df):
        """Find the most likely data column in a DataFrame"""
        # First check for columns named 'data'
        if 'data' in df.columns:
            return 'data'
        
        # Otherwise, look for numeric columns that aren't 'Time' or 'Header'
        numeric_cols = [col for col in df.columns if 
                         col not in ['Time', 'Header'] and 
                         pd.api.types.is_numeric_dtype(df[col])]
        
        if numeric_cols:
            return numeric_cols[0]
        
        # If no numeric columns, try to convert a string column
        obj_cols = [col for col in df.columns if col not in ['Time', 'Header']]
        for col in obj_cols:
            try:
                # Check if we can convert to numeric
                test = pd.to_numeric(df[col], errors='coerce')
                if not test.isna().all():
                    return col
            except:
                continue
        
        # If all else fails, return None
        return None
    
    def update_plot(self):
        """Update the plot with current settings"""
        if self.bag_reader is None or not self.selected_topics:
            # Clear the figure if no topics to plot
            self.figure.clear()
            self.canvas.draw()
            return
        
        # Clear the figure
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        has_data = False
        
        # Sort topics by name for consistent coloring
        sorted_topics = sorted(self.selected_topics, key=lambda x: x.name)
        
        # Plot each selected topic
        for topic_item in sorted_topics:
            topic = topic_item.name
            color = topic_item.color
            qt_color_to_rgb = lambda c: (c.red()/255, c.green()/255, c.blue()/255)
            color_rgb = qt_color_to_rgb(color)
            
            df = self.get_topic_data(topic)
            
            if df is not None:
                # Find data column
                data_col = self.find_data_column(df)
                
                if data_col and 'Time' in df.columns:
                    time = df['Time']
                    data = df[data_col]
                    
                    # Check if data is string/object and try to convert
                    if data.dtype == 'object':
                        try:
                            # Try to convert string data to numeric
                            data = pd.to_numeric(data, errors='coerce')
                            # Remove NaN values
                            valid_indices = ~data.isna()
                            data = data[valid_indices]
                            time = time[valid_indices]
                        except Exception as e:
                            self.statusBar().showMessage(f"Warning: Could not convert {data_col} to numeric: {str(e)}")
                            continue
                    
                    # Check if we have any data points after conversion
                    if len(data) == 0:
                        self.statusBar().showMessage(f"Warning: No valid data in {topic} column {data_col}")
                        continue
                    
                    has_data = True
                    
                    # Normalize time to start at 0
                    norm_time = time - time.iloc[0]
                    
                    # Calculate sampling frequency
                    if len(time) > 1:
                        fs = 1 / (time.iloc[1] - time.iloc[0])
                    else:
                        fs = 1.0  # Default if we can't calculate
                    
                    # Show raw data if selected
                    if topic_item.show_raw:
                        ax.plot(norm_time, data, label=f'{topic} - Raw', 
                               color=color_rgb, alpha=0.5)
                    
                    # Show filtered data if selected
                    if topic_item.show_filtered:
                        try:
                            filtered_data = self.safe_filter(data, self.cutoff, fs, self.filter_order)
                            ax.plot(norm_time, filtered_data, label=f'{topic} - Filtered', 
                                   color=color_rgb)
                        except Exception as e:
                            self.statusBar().showMessage(f"Filter error: {str(e)}")
        
        # Only customize the plot if we have data
        if has_data:
            # Customize the plot
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.set_title('Data Visualization from ROS Bag File')
            ax.legend()
            ax.grid(True)
        
        # Refresh canvas
        self.canvas.draw()
        self.statusBar().showMessage("Plot updated")
    
    def load_camera_frames(self):
        """Load frames for the selected camera topic"""
        if not self.bag_file:
            QMessageBox.warning(self, "Warning", "Please load a bag file first")
            return
        
        topic = self.camera_combo.currentText()
        if not topic:
            QMessageBox.warning(self, "Warning", "No camera topics available")
            return
        
        self.statusBar().showMessage(f"Loading frames from topic: {topic}")
        
        # Show video tab
        self.tab_widget.setCurrentIndex(1)
        
        # Show a progress dialog
        progress_msg = QMessageBox()
        progress_msg.setIcon(QMessageBox.Information)
        progress_msg.setText("Loading camera frames...")
        progress_msg.setInformativeText(f"Loading frames from {topic}")
        progress_msg.setWindowTitle("Loading Frames")
        progress_msg.setStandardButtons(QMessageBox.NoButton)
        progress_msg.show()
        QApplication.processEvents()  # Force UI update
        
        try:
            # Load frames in the image display widget
            success = self.image_display.load_frames(self.bag_file, topic)
            
            # Close progress dialog
            progress_msg.close()
            
            if success:
                self.statusBar().showMessage(f"Loaded frames from {topic}")
            else:
                self.statusBar().showMessage(f"Failed to load frames from {topic}")
                QMessageBox.warning(self, "Warning", f"Failed to load frames from {topic}")
        except Exception as e:
            # Close progress dialog
            progress_msg.close()
            
            self.statusBar().showMessage(f"Error loading frames: {str(e)}")
            QMessageBox.warning(self, "Warning", f"Failed to load frames: {str(e)}")
            import traceback
            print(f"Error details:")
            print(traceback.format_exc())
    
    def convert_to_video(self):
        """Convert selected camera topic to video file"""
        if not self.bag_file:
            QMessageBox.warning(self, "Warning", "Please load a bag file first")
            return
            
        topic = self.camera_combo.currentText()
        if not topic:
            QMessageBox.warning(self, "Warning", "Please select a camera topic")
            return
            
        # Check if the topic contains camera-related keywords
        if 'usb_cam' not in topic and 'image' not in topic and 'camera' not in topic:
            QMessageBox.warning(self, "Warning", 
                            f"The selected topic '{topic}' may not be compatible with video conversion. "
                            "It's recommended to use camera topics.")
            reply = QMessageBox.question(self, "Continue?", 
                                        "Do you want to try conversion anyway?",
                                        QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No:
                return
        
        # Get output file path
        file_dialog = QFileDialog()
        output_path, _ = file_dialog.getSaveFileName(self, "Save Video File", "", "Video Files (*.avi)")
        
        if not output_path:
            return
            
        # Show processing message
        self.statusBar().showMessage(f"Converting {topic} to video file...")
        QApplication.processEvents()  # Force UI update
        
        try:
            # Verify that the bag file exists
            if not os.path.exists(self.bag_file):
                QMessageBox.critical(self, "Error", f"Bag file not found: {self.bag_file}")
                return
                
            # Show a progress dialog
            progress_msg = QMessageBox()
            progress_msg.setIcon(QMessageBox.Information)
            progress_msg.setText("Converting video...")
            progress_msg.setInformativeText(f"Converting {topic} from {self.bag_file} to {output_path}")
            progress_msg.setWindowTitle("Video Conversion")
            progress_msg.setStandardButtons(QMessageBox.NoButton)
            progress_msg.show()
            QApplication.processEvents()  # Force UI update
            
            # Call standalone function instead of class method
            from tqdm import tqdm
            self.rosbag_to_video(self.bag_file, topic, output_path)
            
            # Close progress dialog
            progress_msg.close()
            
            # Check if the output file was created
            if not os.path.exists(output_path):
                QMessageBox.warning(self, "Warning", 
                                f"Conversion completed but the output file was not found at {output_path}")
                return
                
            # Check file size to ensure it's not empty
            if os.path.getsize(output_path) < 1000:  # Less than 1KB is suspicious
                QMessageBox.warning(self, "Warning", 
                                f"The output file was created but may be empty or corrupted ({os.path.getsize(output_path)} bytes)")
            
            self.statusBar().showMessage(f"Video saved to {output_path}")
            
            # Ask if user wants to preview the converted video
            reply = QMessageBox.question(self, "Preview Video", 
                                        "Video conversion complete. Would you like to preview it?",
                                        QMessageBox.Yes | QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                # Try to open the video file with the default system application
                try:
                    if sys.platform.startswith('darwin'):  # macOS
                        os.system(f'open "{output_path}"')
                    elif sys.platform.startswith('win'):   # Windows
                        os.system(f'start "" "{output_path}"')
                    else:  # Linux
                        os.system(f'xdg-open "{output_path}"')
                except Exception as e:
                    QMessageBox.warning(self, "Warning", 
                                    f"Could not open the video file: {str(e)}")
                    
        except Exception as e:
            self.statusBar().showMessage(f"Error converting video: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to convert video: {str(e)}")
            # Print detailed error information
            import traceback
            print(f"Error details:")
            print(traceback.format_exc())

    def rosbag_to_video(self, bag_file, topic, output_video_file):
        """
        Convert ROS bag camera topic to video file without external dependencies
        Based on the working reference code
        """
        try:
            bag = rosbag.Bag(bag_file, 'r')
            
            images = []
            timestamps = []
            message_count = 0
            
            # First pass to count messages for progress estimation
            for _ in bag.read_messages(topics=[topic]):
                message_count += 1
                
            # Reset the bag
            bag.close()
            bag = rosbag.Bag(bag_file, 'r')
            
            # Process frames
            processed = 0
            for topic_name, msg, t in bag.read_messages(topics=[topic]):
                try:
                    # Update progress every 10 frames
                    if processed % 10 == 0:
                        progress_percent = int(100 * processed / max(1, message_count))
                        self.statusBar().showMessage(f"Processing frames: {processed}/{message_count} ({progress_percent}%)")
                        QApplication.processEvents()  # Keep UI responsive
                    
                    # For compressed image topics
                    if 'compressed' in topic_name:
                        # Directly decode the compressed image data
                        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    else:
                        # For raw image topics
                        if hasattr(msg, 'encoding'):
                            encoding = msg.encoding
                        elif hasattr(msg, 'format'):
                            encoding = msg.format
                        else:
                            encoding = 'rgb8'  # Default assumption
                            
                        # Handle compound format strings
                        if ';' in encoding:
                            encoding_parts = encoding.split(';')
                            if any('compressed' in part for part in encoding_parts):
                                image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                            else:
                                encoding = encoding_parts[0].strip()
                                
                        # Handle non-compressed formats
                        if 'compressed' not in topic_name and ('jpeg' not in encoding.lower() and 'png' not in encoding.lower()):
                            height = msg.height
                            width = msg.width
                            
                            # Handle different color encodings
                            if encoding == 'mono8':
                                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
                                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                            elif encoding == 'rgb8':
                                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                            elif encoding == 'bgr8':
                                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                            else:
                                # Try to guess based on step
                                step = getattr(msg, 'step', width * 3)
                                channels = step // width if width > 0 else 3
                                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, channels))
                                if channels == 3 and encoding.startswith('rgb'):
                                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                    if image is None:
                        print("Failed to decode an image, skipping...")
                        continue

                    images.append(image)
                    timestamps.append(t.to_sec())  # Convert ROS time to seconds
                    processed += 1
                    
                except Exception as e:
                    print(f"Skipping message due to error: {e}")
                    continue

            bag.close()

            if len(images) < 2:
                raise ValueError("Not enough frames to create a video (less than 2 frames processed).")
                
            # Compute frame rates based on time differences
            time_diffs = np.diff(timestamps)  # Compute time gaps between frames
            avg_fps = 1.0 / np.mean(time_diffs)  # Compute the average frame rate
            print(f"Computed FPS: {avg_fps:.2f}")
            
            # Use a reasonable default if the computed FPS is too extreme
            if avg_fps < 1 or avg_fps > 100:
                avg_fps = 30
                print(f"FPS value was extreme, defaulting to {avg_fps}")

            # Get resolution from the first image
            height, width = images[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter(output_video_file, fourcc, avg_fps, (width, height))
            
            # Write video frames
            for i, img in enumerate(images):
                # Update progress
                if i % 10 == 0:
                    progress_percent = int(100 * i / len(images))
                    self.statusBar().showMessage(f"Writing video: {i}/{len(images)} frames ({progress_percent}%)")
                    QApplication.processEvents()
                    
                out.write(img)

            out.release()
            print(f"Video saved to {output_video_file}")
            self.statusBar().showMessage(f"Video saved to {output_video_file} with {len(images)} frames at {avg_fps:.2f} FPS")
            
            return True
            
        except Exception as e:
            raise Exception(f"Error in video conversion: {str(e)}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BagVisualizer()
    window.show()
    sys.exit(app.exec_())