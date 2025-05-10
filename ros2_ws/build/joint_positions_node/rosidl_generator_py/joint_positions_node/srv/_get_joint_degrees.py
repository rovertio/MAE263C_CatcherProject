# generated from rosidl_generator_py/resource/_idl.py.em
# with input from joint_positions_node:srv/GetJointDegrees.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetJointDegrees_Request(type):
    """Metaclass of message 'GetJointDegrees_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('joint_positions_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'joint_positions_node.srv.GetJointDegrees_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_joint_degrees__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_joint_degrees__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_joint_degrees__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_joint_degrees__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_joint_degrees__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetJointDegrees_Request(metaclass=Metaclass_GetJointDegrees_Request):
    """Message class 'GetJointDegrees_Request'."""

    __slots__ = [
        '_dummy',
    ]

    _fields_and_field_types = {
        'dummy': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.dummy = kwargs.get('dummy', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.dummy != other.dummy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def dummy(self):
        """Message field 'dummy'."""
        return self._dummy

    @dummy.setter
    def dummy(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'dummy' field must be of type 'bool'"
        self._dummy = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# Member 'degrees'
import numpy  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetJointDegrees_Response(type):
    """Metaclass of message 'GetJointDegrees_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('joint_positions_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'joint_positions_node.srv.GetJointDegrees_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_joint_degrees__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_joint_degrees__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_joint_degrees__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_joint_degrees__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_joint_degrees__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetJointDegrees_Response(metaclass=Metaclass_GetJointDegrees_Response):
    """Message class 'GetJointDegrees_Response'."""

    __slots__ = [
        '_degrees',
    ]

    _fields_and_field_types = {
        'degrees': 'double[2]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 2),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'degrees' not in kwargs:
            self.degrees = numpy.zeros(2, dtype=numpy.float64)
        else:
            self.degrees = numpy.array(kwargs.get('degrees'), dtype=numpy.float64)
            assert self.degrees.shape == (2, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if all(self.degrees != other.degrees):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def degrees(self):
        """Message field 'degrees'."""
        return self._degrees

    @degrees.setter
    def degrees(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'degrees' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 2, \
                "The 'degrees' numpy.ndarray() must have a size of 2"
            self._degrees = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'degrees' field must be a set or sequence with length 2 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._degrees = numpy.array(value, dtype=numpy.float64)


class Metaclass_GetJointDegrees(type):
    """Metaclass of service 'GetJointDegrees'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('joint_positions_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'joint_positions_node.srv.GetJointDegrees')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_joint_degrees

            from joint_positions_node.srv import _get_joint_degrees
            if _get_joint_degrees.Metaclass_GetJointDegrees_Request._TYPE_SUPPORT is None:
                _get_joint_degrees.Metaclass_GetJointDegrees_Request.__import_type_support__()
            if _get_joint_degrees.Metaclass_GetJointDegrees_Response._TYPE_SUPPORT is None:
                _get_joint_degrees.Metaclass_GetJointDegrees_Response.__import_type_support__()


class GetJointDegrees(metaclass=Metaclass_GetJointDegrees):
    from joint_positions_node.srv._get_joint_degrees import GetJointDegrees_Request as Request
    from joint_positions_node.srv._get_joint_degrees import GetJointDegrees_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
