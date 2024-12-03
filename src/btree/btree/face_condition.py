import time

class FaceChecker:
    def __init__(self, time_threshold=5, range_threshold=0.05, time_cooldown=10):

        self.time_threshold = time_threshold
        self.range_threshold = range_threshold
        self.time_cooldown = time_cooldown
        self.initial_position = None
        self.timer_start = None
        self.is_still = False
        self.new_face_appear = False
        self.last_time_bored = None
        self.last_face_count = 0

    def check_face_bored(self, face_detected, face_position):

        if not face_detected:
            self.timer_start = time.time()
            self.initial_position = None
            self.is_still = False
            return False

        if self.initial_position is None:
            self.initial_position = face_position
            self.timer_start = time.time()
            self.is_still = False
            return False

        # Check if the position has changed significantly
        if not self._is_within_range(self.initial_position, face_position):
            self.initial_position = face_position
            self.timer_start = time.time()
            self.is_still = False
            return False

        # Calculate elapsed time since the timer started
        elapsed_time = time.time() - self.timer_start

        if elapsed_time >= self.time_threshold:
            if self.last_time_bored:
                interval_time = time.time() - self.last_time_bored
                if interval_time < self.time_cooldown:
                    self.is_still = False
                    return False
            # Update last_time_bored when boredom is detected
            self.is_still = True
            self.last_time_bored = time.time()
        else:
            self.is_still = False

        return self.is_still

    def check_face_alert(self,face_count):
        if face_count > self.last_face_count:
            self.new_face_appear = True
        else:
            self.new_face_appear = False
        self.last_face_count = face_count
        return self.new_face_appear
            
    
    def _is_within_range(self, pos1, pos2):

        return all(abs(p1 - p2) <= self.range_threshold for p1, p2 in zip(pos1, pos2))
