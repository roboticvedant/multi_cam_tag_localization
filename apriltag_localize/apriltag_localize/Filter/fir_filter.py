# Filter/fir_filter.py

import numpy as np
from scipy import signal
from collections import deque

class TagFilter:
    def __init__(self, order=5, cutoff_freq=2.0):
        """
        Initialize filter for a single tag
        Args:
            order: Filter order
            cutoff_freq: Cutoff frequency in Hz
        """
        self.order = order
        self.cutoff_freq = cutoff_freq
        self.last_timestamp = None
        self.sample_times = []  # Store last N timestamps for rate calculation
        self.max_time_samples = 10  # Number of samples to average for rate
        self.coeffs = None
        self.buffers = {
            'x': deque(maxlen=order + 1),
            'y': deque(maxlen=order + 1),
            'z': deque(maxlen=order + 1),
            'roll': deque(maxlen=order + 1),
            'pitch': deque(maxlen=order + 1),
            'yaw': deque(maxlen=order + 1)
        }
    
    def _update_sample_rate(self, timestamp):
        """
        Update sampling rate based on detection timestamps
        Args:
            timestamp: Current detection timestamp (in seconds)
        """
        if self.last_timestamp is not None:
            dt = timestamp - self.last_timestamp
            self.sample_times.append(dt)
            if len(self.sample_times) > self.max_time_samples:
                self.sample_times.pop(0)
            
            # Calculate average sampling rate
            avg_dt = sum(self.sample_times) / len(self.sample_times)
            sample_freq = 1.0 / avg_dt if avg_dt > 0 else 30.0  # fallback to 30Hz
            
            # Update filter coefficients
            self.coeffs = signal.firwin(self.order + 1, 
                                      self.cutoff_freq / (sample_freq / 2))
        
        self.last_timestamp = timestamp

    def filter_value(self, value, buffer_name, timestamp):
        """
        Filter a single value using the specified buffer
        Args:
            value: New value to filter
            buffer_name: Name of the buffer to use ('x', 'y', 'z', etc.)
            timestamp: Current detection timestamp
        Returns:
            Filtered value or original value if buffer not full
        """
        # Update sampling rate and coefficients
        self._update_sample_rate(timestamp)
        
        # If coefficients haven't been initialized yet, do it now
        if self.coeffs is None:
            sample_freq = 30.0  # Initial guess
            self.coeffs = signal.firwin(self.order + 1, 
                                      self.cutoff_freq / (sample_freq / 2))
        
        # Add value to buffer
        self.buffers[buffer_name].append(value)
        
        # Apply filter if buffer is full
        if len(self.buffers[buffer_name]) == self.buffers[buffer_name].maxlen:
            return np.sum(np.array(self.buffers[buffer_name]) * self.coeffs)
        return value  # Return original value if buffer not full

    def reset(self):
        """Reset filter buffers and timing information"""
        for buffer in self.buffers.values():
            buffer.clear()
        self.last_timestamp = None
        self.sample_times.clear()
        self.coeffs = None