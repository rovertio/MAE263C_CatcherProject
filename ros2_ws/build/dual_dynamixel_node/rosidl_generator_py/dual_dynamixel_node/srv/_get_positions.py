# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dual_dynamixel_node:srv/GetPositions.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'id'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetPositions_Request(type):
    """Metaclass of message 'GetPositions_Request'."""

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
            module = import_type_support('dual_dynamixel_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dual_dynamixel_node.srv.GetPositions_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_positions__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_positions__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_positions__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_positions__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_positions__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetPositions_Request(metaclass=Metaclass_GetPositions_Request):
    """Message class 'GetPositions_Request'."""

    __slots__ = [
        '_id',
    ]

    _fields_and_field_types = {
        'id': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = array.array('B', kwargs.get('id', []))

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
        if self.id != other.id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'id' array.array() must have the type code of 'B'"
            self._id = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'id' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._id = array.array('B', value)


# Import statements for member types

# Member 'position'
# already imported above
# import array

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetPositions_Response(type):
    """Metaclass of message 'GetPositions_Response'."""

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
            module = import_type_support('dual_dynamixel_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dual_dynamixel_node.srv.GetPositions_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_positions__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_positions__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_positions__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_positions__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_positions__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetPositions_Response(metaclass=Metaclass_GetPositions_Response):
    """Message class 'GetPositions_Response'."""

    __slots__ = [
        '_position',
    ]

    _fields_and_field_types = {
        'position': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.position = array.array('i', kwargs.get('position', []))

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
        if self.position != other.position:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'position' array.array() must have the type code of 'i'"
            self._position = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'position' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._position = array.array('i', value)


class Metaclass_GetPositions(type):
    """Metaclass of service 'GetPositions'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dual_dynamixel_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dual_dynamixel_node.srv.GetPositions')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_positions

            from dual_dynamixel_node.srv import _get_positions
            if _get_positions.Metaclass_GetPositions_Request._TYPE_SUPPORT is None:
                _get_positions.Metaclass_GetPositions_Request.__import_type_support__()
            if _get_positions.Metaclass_GetPositions_Response._TYPE_SUPPORT is None:
                _get_positions.Metaclass_GetPositions_Response.__import_type_support__()


class GetPositions(metaclass=Metaclass_GetPositions):
    from dual_dynamixel_node.srv._get_positions import GetPositions_Request as Request
    from dual_dynamixel_node.srv._get_positions import GetPositions_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
