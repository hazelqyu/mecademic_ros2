import time

class ConditionChecker:
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):

        self.time_threshold = 5
        self.range_threshold = 0.05
        self.time_cooldown = 10
        self.initial_position = None
        self.timer_start = None
        self.is_still = False
        self.new_face_appear = False
        self.last_execution_time = None
        self.last_face_count = 0
        self.face_lost_time = time.time()

    def check_face_still(self, face_detected, face_position):

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
            # if self.last_execution_time:
            #     interval_time = time.time() - self.last_execution_time
            #     if interval_time < self.time_cooldown:
            #         self.is_still = False
            #         return False
            self.is_still = True
            # self.last_execution_time = time.time()
        else:
            self.is_still = False

        return self.is_still

    def check_scanning(self,face_detected):
        if face_detected:
            self.face_lost_time = time.time()
            return True
        elapsed_time = time.time()-self.face_lost_time
        if elapsed_time < 15:
            return True
        return False

    def check_face_appear(self,face_count):
        if face_count > self.last_face_count:
            self.new_face_appear = True
        else:
            self.new_face_appear = False
        self.last_face_count = face_count
        return self.new_face_appear
    
    def update_last_exe_time(self):
        self.last_execution_time = time.time()
        
    def check_face_too_close(self, closest_face):
        if not closest_face:
            return False
        if closest_face['depth']<0.6:
            return True
        else:
            return False
        
    
    def _is_within_range(self, pos1, pos2):
        return all(abs(p1 - p2) <= self.range_threshold for p1, p2 in zip(pos1, pos2))
