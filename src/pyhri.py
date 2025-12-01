"""
Python wrapper library to the ROS4HRI (https://www.ros.org/reps/rep-0155.html) framework.

Each exported object is documented, to view it use `print(<class>.__doc__)` and/or
`help(<class)`.
The main entry point for its usage is :py:class:`HRIListener` class.
"""

from enum import Enum
from typing import Optional, Dict, List, Tuple, Callable
import geometry_msgs.msg
import numpy

class FeatureTracker:
    """
    The generic feature instance being tracked.

    This class should be created and managed only by :py:class`HRIListener`, it is exposed
    only for read access purposes. All its properties may return None if not
    available.
    """
    @property
    def id(self) -> str:
        """Unique ID of this feature"""
        return ""

    @property
    def ns(self) -> str:
        """Fully-qualified topic namespace under which this feature is published"""
        return ""

    @property
    def frame(self) -> str:
        """Name of the tf frame that correspond to this feature"""
        return ""

    @property
    def transform(self) -> Optional[geometry_msgs.msg.TransformStamped]:
        """Feature stamped 3D transform (geometry_msgs.msg.TransformStamped)"""
        return None

    @property
    def valid(self) -> bool:
        """Whether the feature is still 'valid', i.e., existing"""
        return False

class Body(FeatureTracker):
    """
    The body feature instance being tracked.

    This class should be created and managed only by :py:class`HRIListener`, it is exposed only for read access
    purposes.
    It inherits from :py:class:`FeatureTracker`, check its documentation for additional properties.
    All its properties may return None if not available.
    """
    @property
    def roi(self) -> Optional[Tuple[float, float, float, float]]:
        """Normalized 2D region of interest (RoI) of the body (Tuple (x,y,width,height))"""
        return None

    @property
    def cropped(self) -> Optional[numpy.ndarray]:
        """Body image, cropped from the source image (numpy.ndarray)"""
        return None

    @property
    def skeleton(self) -> Optional[Dict['SkeletalKeypoint', 'PointOfInterest']]:
        """2D skeleton keypoints (Dict[SkeletalKeypoint, PointOfInterest])"""
        return None

class Face(FeatureTracker):
    """
    The face feature instance being tracked.

    This class should be created and managed only by :py:class`HRIListener`, it is exposed only for read access
    purposes.
    It inherits from :py:class:`FeatureTracker`, check its documentation for additional properties.
    All its properties may return None if not available.
    """
    @property
    def roi(self) -> Optional[Tuple[float, float, float, float]]:
        """Normalized 2D region of interest (RoI) of the face (Tuple (x,y,width,height))"""
        return None

    @property
    def cropped(self) -> Optional[numpy.ndarray]:
        """Face image, cropped from the source image (numpy.ndarray)"""
        return None

    @property
    def aligned(self) -> Optional[numpy.ndarray]:
        """Face image, cropped and aligned from the source image (numpy.ndarray)"""
        return None

    @property
    def facial_landmarks(self) -> Optional[Dict['FacialLandmark', 'PointOfInterest']]:
        """Facial landmarks (Dict[FacialLandmark, PointOfInterest])"""
        return None

    @property
    def facial_action_units(self) -> Optional[Dict['FacialActionUnit', 'IntensityConfidence']]:
        """Facial action units (Dict[FacialActionUnit, IntensityConfidence])"""
        return None

    @property
    def age(self) -> Optional[float]:
        """Person's age in years (float)"""
        return None

    @property
    def gender(self) -> Optional['Gender']:
        """Person's gender (Gender)"""
        return None

    @property
    def expression(self) -> Optional['Expression']:
        """Face expression as a discrete state"""
        return None

    @property
    def expression_va(self) -> Optional['ExpressionVA']:
        """Face expression as a continuous value in the circumplex model space (ExpressionVA)"""
        return None

    @property
    def expression_confidence(self) -> Optional[float]:
        """Person's expression confidence"""
        return None

    @property
    def gaze_transform(self) -> Optional[geometry_msgs.msg.TransformStamped]:
        """Gaze's stamped 3D transform (geometry_msgs.msg.TransformStamped)"""
        return None

class Voice(FeatureTracker):
    """
    The voice feature instance being tracked.

    This class should be created and managed only by :py:class`HRIListener`, it is exposed only for read access
    purposes.
    It inherits from :py:class:`FeatureTracker`, check its documentation for additional properties.
    All its properties may return None if not available.
    """
    @property
    def is_speaking(self) -> bool:
        """Whether speech is currently detected in this voice (bool)"""
        return False

    @property
    def speech(self) -> Optional[str]:
        """Last recognised final sentence (str)"""
        return None

    @property
    def incremental_speech(self) -> Optional[str]:
        """Last recognised incremental sentence (str)"""
        return None

    @property
    def locale(self) -> Optional[str]:
        """Last recognised speech locale (str)"""
        return None

    def on_speaking(self, callback: Callable[[bool], None]):
        """Registers a callback function, to be invoked everytime speech is detected"""
        pass

    def on_speech(self, callback: Callable[[str, str], None]):
        """Registers a callback function, to be invoked everytime a final sentence is detected"""
        pass

    def on_incremental_speech(self, callback: Callable[[str, str], None]):
        """Registers a callback function, to be invoked everytime an incremental sentence is detected"""
        pass

class Person(FeatureTracker):
    """
    The person feature instance being tracked or known.

    This class should be created and managed only by :py:class`HRIListener`, it is exposed only for read access
    purposes.
    It inherits from :py:class:`FeatureTracker`, check its documentation for additional properties.
    All its properties may return None if not available.
    """
    @property
    def face(self) -> Optional[Face]:
        """Face associated with the person (Face)"""
        return None

    @property
    def body(self) -> Optional[Body]:
        """Body associated with the person (Body)"""
        return None

    @property
    def voice(self) -> Optional[Voice]:
        """Voice associated with the person (Voice)"""
        return None

    @property
    def anonymous(self) -> bool:
        """Whether the person has not been identified yet (bool)"""
        return False

    @property
    def engagement_status(self) -> Optional['EngagementLevel']:
        """Current engagement status with the robot (EngagementLevel)"""
        return None

    @property
    def location_confidence(self) -> float:
        """confidence of the person transform estimate (float [0., 1.])"""
        return 0.0

    @property
    def alias(self) -> Optional[str]:
        """ID of another Person object associated with the same person (str)"""
        return None

class HRIListener:
    """
    Main entry point to the library.

    The class must be instantiated through the factory function ``create``.
    I will spawn a ROS node and use it to subscribe to all the ROS4HRI topics.
    The tracked features information can be accessed in Python native objects throught this object
    properties.
    """
    def __init__(self, node_name: str, auto_spin: bool = True, use_sim_time: bool = False):
        """
        Generate the class, selecting the spawned node name and whether it spins automatically
        """
        pass

    @property
    def faces(self) -> Dict[str, Face]:
        """Currently tracked faces (Dict[str, Face])"""
        return {}

    @property
    def bodies(self) -> Dict[str, Body]:
        """Currently tracked bodies (Dict[str, Body])"""
        return {}

    @property
    def voices(self) -> Dict[str, Voice]:
        """Currently tracked voices (Dict[str, Voice])"""
        return {}

    @property
    def persons(self) -> Dict[str, Person]:
        """Currently known persons (Dict[str, Person])"""
        return {}

    @property
    def tracked_persons(self) -> Dict[str, Person]:
        """Currently tracked persons (Dict[str, Person])"""
        return {}

    def on_face(self, callback: Callable[[Face], None]):
        """Registers a callback function, to be invoked everytime a new face is tracked"""
        pass

    def on_body(self, callback: Callable[[Body], None]):
        """Registers a callback function, to be invoked everytime a new body is tracked"""
        pass

    def on_voice(self, callback: Callable[[Voice], None]):
        """Registers a callback function, to be invoked everytime a new voice is tracked"""
        pass

    def on_person(self, callback: Callable[[Person], None]):
        """Registers a callback function, to be invoked everytime a new person is known"""
        pass

    def on_tracked_person(self, callback: Callable[[Person], None]):
        """Registers a callback function, to be invoked everytime a new person is tracked"""
        pass

    def on_face_lost(self, callback: Callable[[str], None]):
        """Registers a callback function, to be invoked everytime a tracked face is lost"""
        pass

    def on_body_lost(self, callback: Callable[[str], None]):
        """Registers a callback function, to be invoked everytime a tracked body is lost"""
        pass

    def on_voice_lost(self, callback: Callable[[str], None]):
        """Registers a callback function, to be invoked everytime a tracked voice is lost"""
        pass

    def on_person_lost(self, callback: Callable[[str], None]):
        """Registers a callback function, to be invoked everytime a known person is forgotten"""
        pass

    def on_tracked_person_lost(self, callback: Callable[[str], None]):
        """Registers a callback function, to be invoked everytime a tracked person is lost"""
        pass

    def set_reference_frame(self, frame: str):
        """Selects the reference frame for all the `transform` properties"""
        pass

    def spin_all(self, timeout: float):
        """
        If the class node does not spin automatically, this function must be called
        regularly to manually spin it.
        Internally calls rclcpp::executors::SingleThreadedExecutor::spin_all()
        """
        pass

class EngagementLevel(Enum):
    DISENGAGED = 0
    ENGAGING = 1
    ENGAGED = 2
    DISENGAGING = 3

class Expression(Enum):
    NEUTRAL = 0
    ANGRY = 1
    SAD = 2
    HAPPY = 3
    SURPRISED = 4
    DISGUSTED = 5
    SCARED = 6
    PLEADING = 7
    VULNERABLE = 8
    DESPAIRED = 9
    GUILTY = 10
    DISAPPOINTED = 11
    EMBARRASSED = 12
    HORRIFIED = 13
    SKEPTICAL = 14
    ANNOYED = 15
    FURIOUS = 16
    SUSPICIOUS = 17
    REJECTED = 18
    BORED = 19
    TIRED = 20
    ASLEEP = 21
    CONFUSED = 22
    AMAZED = 23
    EXCITED = 24

class Gender(Enum):
    FEMALE = 0
    MALE = 1
    OTHER = 2

class FacialActionUnit(Enum):
    NEUTRAL_FACE = 0
    INNER_BROW_RAISER = 1
    OUTER_BROW_RAISER = 2
    BROW_LOWERER = 4
    UPPER_LID_RAISER = 5
    CHEEK_RAISER = 6
    LID_TIGHTENER = 7
    LIPS_TOWARD_EACH_OTHER = 8
    NOSE_WRINKLER = 9
    UPPER_LIP_RAISER = 10
    NASOLABIAL_DEEPENER = 11
    LIP_CORNER_PULLER = 12
    SHARP_LIP_PULLER = 13
    DIMPLER = 14
    LIP_CORNER_DEPRESSOR = 15
    LOWER_LIP_DEPRESSOR = 16
    CHIN_RAISER = 17
    LIP_PUCKER = 18
    TONGUE_SHOW = 19
    LIP_STRETCHER = 20
    NECK_TIGHTENER = 21
    LIP_FUNNELER = 22
    LIP_TIGHTENER = 23
    LIP_PRESSOR = 24
    LIPS_PART = 25
    JAW_DROP = 26
    MOUTH_STRETCH = 27
    LIP_SUCK = 28
    HEAD_TURN_LEFT = 51
    HEAD_TURN_RIGHT = 52
    HEAD_UP = 53
    HEAD_DOWN = 54
    HEAD_TILT_LEFT = 55
    HEAD_TILT_RIGHT = 56
    HEAD_FORWARD = 57
    HEAD_BACK = 58
    EYES_TURN_LEFT = 61
    EYES_TURN_RIGHT = 62
    EYES_UP = 63
    EYES_DOWN = 64
    WALLEYE = 65
    CROSS_EYE = 66
    EYES_POSITIONED_TO_LOOK_AT_OTHER_PERSON = 69
    BROWS_AND_FOREHEAD_NOT_VISIBLE = 70
    EYES_NOT_VISIBLE = 71
    LOWER_FACE_NOT_VISIBLE = 72
    ENTIRE_FACE_NOT_VISIBLE = 73
    UNSOCIABLE = 74
    JAW_THRUST = 29
    JAW_SIDEWAYS = 30
    JAW_CLENCHER = 31
    LIP_BITE = 32
    CHEEK_BLOW = 33
    CHEEK_PUFF = 34
    CHEEK_SUCK = 35
    TONGUE_BULGE = 36
    LIP_WIPE = 37
    NOSTRIL_DILATOR = 38
    NOSTRIL_COMPRESSOR = 39
    SNIFF = 40
    LID_DROOP = 41
    SLIT = 42
    EYES_CLOSED = 43
    SQUINT = 44
    BLINK = 45
    WINK = 46
    SPEECH = 50
    SWALLOW = 80
    CHEWING = 81
    SHOULDER_SHRUG = 82
    HEAD_SHAKE_BACK_AND_FORTH = 83
    HEAD_NOD_UP_AND_DOWN = 84
    FLASH = 91
    PARTIAL_FLASH = 92
    SHIVER_TREMBLE = 93
    FAST_UP_DOWN_LOOK = 94

class FacialLandmark(Enum):
    RIGHT_EAR = 0
    RIGHT_PROFILE_1 = 1
    RIGHT_PROFILE_2 = 2
    RIGHT_PROFILE_3 = 3
    RIGHT_PROFILE_4 = 4
    RIGHT_PROFILE_5 = 5
    RIGHT_PROFILE_6 = 6
    RIGHT_PROFILE_7 = 7
    MENTON = 8
    LEFT_EAR = 9
    LEFT_PROFILE_1 = 10
    LEFT_PROFILE_2 = 11
    LEFT_PROFILE_3 = 12
    LEFT_PROFILE_4 = 13
    LEFT_PROFILE_5 = 14
    LEFT_PROFILE_6 = 15
    LEFT_PROFILE_7 = 16
    RIGHT_EYEBROW_OUTSIDE = 17
    RIGHT_EYEBROW_1 = 18
    RIGHT_EYEBROW_2 = 19
    RIGHT_EYEBROW_3 = 20
    RIGHT_EYEBROW_INSIDE = 21
    RIGHT_EYE_OUTSIDE = 22
    RIGHT_EYE_TOP_1 = 23
    RIGHT_EYE_TOP_2 = 24
    RIGHT_EYE_INSIDE = 25
    RIGHT_EYE_BOTTOM_1 = 26
    RIGHT_EYE_BOTTOM_2 = 27
    RIGHT_PUPIL = 28
    LEFT_EYEBROW_OUTSIDE = 29
    LEFT_EYEBROW_1 = 30
    LEFT_EYEBROW_2 = 31
    LEFT_EYEBROW_3 = 32
    LEFT_EYEBROW_INSIDE = 33
    LEFT_EYE_OUTSIDE = 34
    LEFT_EYE_TOP_1 = 35
    LEFT_EYE_TOP_2 = 36
    LEFT_EYE_INSIDE = 37
    LEFT_EYE_BOTTOM_1 = 38
    LEFT_EYE_BOTTOM_2 = 39
    LEFT_PUPIL = 40
    SELLION = 41
    NOSE_1 = 42
    NOSE_2 = 43
    NOSE = 44
    NOSTRIL_1 = 45
    NOSTRIL_2 = 46
    NOSTRIL_3 = 47
    NOSTRIL_4 = 48
    NOSTRIL_5 = 49
    MOUTH_OUTER_RIGHT = 50
    MOUTH_OUTER_TOP_1 = 51
    MOUTH_OUTER_TOP_2 = 52
    MOUTH_OUTER_TOP_3 = 53
    MOUTH_OUTER_TOP_4 = 54
    MOUTH_OUTER_TOP_5 = 55
    MOUTH_OUTER_LEFT = 56
    MOUTH_OUTER_BOTTOM_1 = 57
    MOUTH_OUTER_BOTTOM_2 = 58
    MOUTH_OUTER_BOTTOM_3 = 59
    MOUTH_OUTER_BOTTOM_4 = 60
    MOUTH_OUTER_BOTTOM_5 = 61
    MOUTH_INNER_RIGHT = 62
    MOUTH_INNER_TOP_1 = 63
    MOUTH_INNER_TOP_2 = 64
    MOUTH_INNER_TOP_3 = 65
    MOUTH_INNER_LEFT = 66
    MOUTH_INNER_BOTTOM_1 = 67
    MOUTH_INNER_BOTTOM_2 = 68
    MOUTH_INNER_BOTTOM_3 = 69

class SkeletalKeypoint(Enum):
    NOSE = 0
    NECK = 1
    RIGHT_SHOULDER = 2
    RIGHT_ELBOW = 3
    RIGHT_WRIST = 4
    LEFT_SHOULDER = 5
    LEFT_ELBOW = 6
    LEFT_WRIST = 7
    RIGHT_HIP = 8
    RIGHT_KNEE = 9
    RIGHT_ANKLE = 10
    LEFT_HIP = 11
    LEFT_KNEE = 12
    LEFT_ANKLE = 13
    LEFT_EYE = 14
    RIGHT_EYE = 15
    LEFT_EAR = 16
    RIGHT_EAR = 17
