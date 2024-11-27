import time

class FaceChecker:
    def __init__(self, time_threshold=5, range_threshold=0.05):

        self.time_threshold = time_threshold
        self.range_threshold = range_threshold
        self.initial_position = None
        self.timer_start = None
        self.is_still = False

    def check_face_bored(self, face_detected,face_position):
        
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
            # Significant change: Reset timer and reference position
            self.initial_position = face_position
            self.timer_start = time.time()
            self.is_still = False
            return False

        # Calculate elapsed time
        elapsed_time = time.time() - self.timer_start
        if elapsed_time >= self.time_threshold:
            self.is_still = True
            return True

        self.is_still = False
        return False

    def _is_within_range(self, pos1, pos2):
        return all(abs(p1 - p2) <= self.range_threshold for p1, p2 in zip(pos1, pos2))
