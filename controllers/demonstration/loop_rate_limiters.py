#!/usr/bin/env python3
"""
Simple rate limiter for controlling simulation loop frequency
"""

import time


class RateLimiter:
    """Simple rate limiter to maintain consistent loop frequency"""
    
    def __init__(self, frequency):
        """
        Initialize rate limiter
        
        Args:
            frequency: Target frequency in Hz
        """
        self.frequency = frequency
        self.dt = 1.0 / frequency
        self.last_time = time.time()
    
    def sleep(self):
        """Sleep to maintain target frequency"""
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed < self.dt:
            time.sleep(self.dt - elapsed)
        
        self.last_time = time.time()