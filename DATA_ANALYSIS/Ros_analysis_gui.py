import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QComboBox, QCheckBox, QLabel, QPushButton, 
                            QSlider, QGroupBox, QFileDialog, QListWidget, QSplitter,
                            QMessageBox, QColorDialog, QListWidgetItem)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QColor, QIcon, QPixmap
from bagpy import bagreader
from scipy.signal import butter, sosfiltfilt
import random

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
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        
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
        splitter.addWidget(left_panel)
        splitter.addWidget(self.plot_widget)
        
        # Set initial sizes
        splitter.setSizes([300, 900])
        
        # Add splitter to main layout
        main_layout.addWidget(splitter)
        
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
            
            # Get available topics
            topic_table = self.bag_reader.topic_table
            self.available_topics = list(topic_table['Topics'])
            
            # Update topic selection combobox
            self.topic_combo.clear()
            for topic in self.available_topics:
                self.topic_combo.addItem(topic)
            
            self.statusBar().showMessage("Bag file loaded successfully")
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BagVisualizer()
    window.show()
    sys.exit(app.exec_())