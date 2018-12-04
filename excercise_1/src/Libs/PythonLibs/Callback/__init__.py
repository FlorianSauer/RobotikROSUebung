"""
Module for better handling Callbacks with arguments.

created by sauer
"""

import inspect
from threading import Lock

from typing import Any, Tuple, Callable, Dict, Generic, TypeVar, Optional

ARGS = TypeVar('ARGS')
R = TypeVar('R')


class Callback(Generic[ARGS, R]):
    """
    Wrapper class for a callback function.

    Ensures that callbacks are being called with the right amount of arguments.
    """

    def __init__(self, func, args=(), kwargs=None, name=None, single=False, once=False):
        # type: (Callable[[ARGS], R], Tuple, Optional[Dict[str, Any]], str, bool, bool) -> None
        """

        :param func: function to call, first parameters MUST handle call values
        :param args: arguments to insert after the call arguments
        :param kwargs: keyword arguments to insert after the positional arguments
        :param name: the callback name; if no name is provided, the function name will be used
        :param single: if this callback can be run parallel or not
        :param once: if this callback can only be called once
        :raises CallbackArgumentCountError: Given function cannot get called, too much arguments passed
        :raises CallbackKeywordArgumentError: A given keyword argument does not exist in the given function
        """

        if kwargs is None:
            kwargs = {}
        if name is None:
            name = func.__name__

        self.__mutex = Lock()
        self.__single_lock = Lock()
        self._activated = True
        self._func = func  # type: Callable[[ARGS], R]
        self._got_called = False  # type: bool
        self._last_result = None  # type: R
        self.single = single  # type: bool
        self.once = once  # type: bool
        self.name = name  # type: str

        spec = inspect.getargspec(self._func)
        if spec.args:
            spec_argscount = len(spec.args)
            self._func_arg_kwarg_names = list(spec.args)
        else:
            self._func_arg_kwarg_names = []
            spec_argscount = 0
        if spec.defaults:
            spec_defaultscount = len(spec.defaults)
        else:
            spec_defaultscount = 0
        self._func_arg_kwarg_count = spec_argscount
        self._func_arg_count = spec_argscount - spec_defaultscount
        self._func_kwarg_count = spec_defaultscount
        self._is_object_func = inspect.ismethod(self._func)
        if self._is_object_func:
            self._func_arg_names = self._func_arg_kwarg_names[1:self._func_arg_count]
            self._func_arg_kwarg_names.remove('self')
            self._func_arg_count -= 1
            self._func_arg_kwarg_count -= 1
        else:
            self._func_arg_names = self._func_arg_kwarg_names[:self._func_arg_count]
        self._func_kwarg_names = self._func_arg_kwarg_names[-self._func_kwarg_count:]
        self._required_arg_count = self._func_arg_count
        if self._required_arg_count < 0:
            self._required_arg_count = 0
        self._required_kwarg_count = self._func_kwarg_count - len(kwargs)
        if self._required_kwarg_count < 0:
            self._required_kwarg_count = 0

        if len(args) + len(kwargs) > self._func_arg_kwarg_count:
            raise CallbackArgumentCountError("Given function cannot get called, too much arguments passed")

        for key in kwargs:
            if key not in spec.args:
                raise CallbackKeywordArgumentError("The given function does not have the keyword argument " + repr(key))

        # expand args to kwargs
        args, kwargs = self._expand_args_kwargs(args, kwargs)

        for argname, argval in zip(reversed(self._func_arg_names), reversed(args)):
            kwargs[argname] = argval
        for argname in kwargs:
            if argname in self._func_arg_names:
                if self._required_arg_count > 0:
                    self._required_arg_count -= 1
        args = ()

        self._args = args  # type: Tuple[Any]
        self._kwargs = kwargs  # type: Dict[str, Any]

    def _expand_args_kwargs(self, args, kwargs):
        if len(args) > self._func_arg_count:
            _args = args[:self._func_arg_count]
            _args_to_kwargs = args[self._func_arg_count:]
            if len(_args_to_kwargs) > len(self._func_kwarg_names):  # we already checked this, should not trigger
                raise NotImplementedError(
                    'length of expandable arguments is longer than there are possible kwarg names')
            for arg_val, kwarg_name in zip(_args_to_kwargs, self._func_kwarg_names):
                if kwarg_name not in kwargs:
                    kwargs[kwarg_name] = arg_val
                else:
                    raise CallbackKeywordArgumentError("got multiple values for keyword argument " + repr(kwarg_name))
            args = _args
        elif any([k in self._func_arg_names for k in kwargs]):
            _kwargs = {}
            _args = list(args)
            for k in self._func_kwarg_names:
                if k in kwargs:
                    _kwargs[k] = kwargs[k]
            for a in self._func_arg_names:
                if a in _kwargs:
                    raise NotImplementedError("arg name already added to new kwargs")
                if a in kwargs:
                    _args.append(kwargs[a])
            args = tuple(_args)
            kwargs = _kwargs
        return args, kwargs

    def activate(self):
        # type: () -> None
        """
        Allows the Callback to be called
        """
        with self.__mutex:
            self._activated = True

    def deactivate(self):
        # type: () -> None
        """
        Disallows the Callback to be called. Calling call() will raise a CallbackDisabledError.
        """
        with self.__mutex:
            self._activated = False

    def is_activated(self):
        # type: () -> bool
        """
        :return: If the Callback is Activated or Deactivated.
        """
        with self.__mutex:
            return self._activated

    def got_called(self):
        # type: () -> bool
        """
        :return: If the Callback has been called yet.
        """
        with self.__mutex:
            return self._got_called

    def last_result(self):
        # type: () -> R
        """
        :return: The return value of the last stored function call. If not called yet it returns None.
        """
        with self.__mutex:
            return self._last_result

    def reset(self):
        # type: () -> None
        """
        Sets the last called value to None and re-allows a call if the Callback is in once-mode
        """
        with self.__mutex:
            self._last_result = None
            self._got_called = False
            self._activated = True

    def callable(self, *args, **kwargs):
        # type: (Tuple[Any], Dict[str, Any]) -> bool
        """
        Checks if the stored method can be called without raising a CallbackArgumentCountError

        :param args: args you would pass to the call() method and thus to the internal stored method
        :param kwargs: like the args argument
        :return: if enough parameters were given
        """
        try:
            self._callable(*args, **kwargs)
            return True
        except CallbackError:
            return False

    def callable_ex(self, *args, **kwargs):
        # type: (Tuple[Any], Dict[str, Any]) -> None
        """
        Like callable(), but instead of returning False, raise a CallbackError exception.

        :param args: args you would pass to the call() method and thus to the internal stored method
        :param kwargs: like the args argument
        """
        self._callable(*args, **kwargs)

    def _callable(self, *args, **kwargs):
        # type: (Tuple[Any], Dict[str, Any]) -> None
        with self.__mutex:
            if not self._activated:
                raise CallbackDisabledError("Callback is disabled. Calling is forbidden.")
        with self.__mutex:
            if self.once and self._got_called:
                raise CallbackAlreadyCalledError("Callback was already Called. No further calls allowed.")
        if args:
            args_count = len(args)
        else:
            args_count = 0
        if kwargs:
            kwargs_count = len(kwargs.keys())
            for key in kwargs:
                if key not in self._func_arg_kwarg_names:
                    raise CallbackKeywordArgumentError(
                        "The given function does not have the keyword argument " + repr(key))

        else:
            kwargs_count = 0
        if args_count > self._required_arg_count:
            raise CallbackArgumentCountError("too much positional arguments passed")
        for k in kwargs:
            if k in self._func_arg_names:
                args_count += 1
                kwargs_count -= 1
        args_kwargs_count = args_count + kwargs_count
        if args_kwargs_count + len(self._args) + len(self._kwargs) > self._func_arg_kwarg_count:
            raise CallbackArgumentCountError("too much arguments passed")
        if args_count + len(self._args) + len(set(kwargs.keys()).intersection(set(self._func_arg_names))) + len(
                set(self._kwargs.keys()).intersection(set(self._func_arg_names))) < self._func_arg_count:
            raise CallbackArgumentCountError("Not enough args and kwargs passed. Expected at least "
                                             + str(self._required_arg_count)
                                             + " arguments, only "
                                             + str(args_count)
                                             + " given")

        if args_count < self._required_arg_count:
            if kwargs_count > 0:
                if len(set(kwargs.keys()).intersection(set(self._func_arg_names))) > 0:
                    pass
                else:
                    raise CallbackArgumentCountError("Not enough args and kwargs passed. Expected at least "
                                                     + str(self._required_arg_count)
                                                     + " arguments, only "
                                                     + str(args_count)
                                                     + " given")

            else:
                raise CallbackArgumentCountError("Not enough args and kwargs passed. Expected at least "
                                                 + str(self._required_arg_count)
                                                 + " arguments, only "
                                                 + str(args_count)
                                                 + " given")

    def call(self, *args, **kwargs):
        # type: (Tuple[Any], Dict[str, Any]) -> R
        """
        Calls the stored function with optional given arguments.

        If the function was not provided with enough parameters, the remaining parameters must be provided as
        positional arguments.
        Given Keyword arguments will replace stored keyword and positional arguments for a single function call.
        Its not possible to provide keyword arguments as positional arguments.

        :raises CallbackArgumentCountError: The number of passed arguments was incorrect.
        :raises CallbackDisabledError: The Callback is disabled. Calling is forbidden.
        :raises CallbackAlreadyCalledError: The Callback was already called. No further calls allowed.
        :raises CallbackKeywordArgumentError: The Callback does not have a given keyword argument
        """
        try:
            self._callable(*args, **kwargs)
        except CallbackError:
            raise
        if self.single:
            with self.__single_lock:
                return self._call(*args, **kwargs)
        else:
            return self._call(*args, **kwargs)

    def _call(self, *args, **kwargs):
        # type: (Tuple, Dict[str, Any]) -> R

        _kwargs = dict(self._kwargs)
        _args = list(args + self._args)

        for param_name, arg in zip(self._func_arg_kwarg_names, _args):
            if param_name in kwargs:
                _kwargs[param_name] = kwargs[param_name]
            else:
                _kwargs[param_name] = arg
        for param_name in self._func_arg_kwarg_names:
            if param_name in kwargs:
                _kwargs[param_name] = kwargs[param_name]
        return_value = self._func(**_kwargs)
        with self.__mutex:
            self._got_called = True
            self._last_result = return_value
            if self.once:
                self._activated = False
        return return_value

    def __repr__(self):
        if self._args and self._kwargs:
            args_kwargs_str = ', '.join([repr(arg) for arg in self._args]) + ", " + ', '.join(
                [str(k) + '=' + repr(self._kwargs[k]) for k in self._func_arg_kwarg_names if k in self._kwargs])
        else:
            args_kwargs_str = ', '.join([repr(arg) for arg in self._args]) + ', '.join(
                [str(k) + '=' + repr(self._kwargs[k]) for k in self._func_arg_kwarg_names if k in self._kwargs])
        if self._required_arg_count == 0:
            return "<Callback " + self.name + "(" + args_kwargs_str + ")>"
        else:
            variable_param_str = ', '.join(['*'] * self._required_arg_count)
            param_str = variable_param_str
            if args_kwargs_str:
                param_str += ", " + args_kwargs_str
            return "<Callback " + self.name + "(" + param_str + ")>"

    def __eq__(self, other):
        # type: (Callback) -> bool
        if not issubclass(type(other), Callback):
            return False
        else:
            return self.name == other.name


class CallbackError(Exception):
    """
    Basic Callback Error
    """
    pass


class CallbackArgumentCountError(CallbackError):
    """
    Error for using the wrong number of arguments during initialisation or calling
    """
    pass


class CallbackKeywordArgumentError(CallbackError):
    """
    Error for passing a not existent kwarg to the stored function
    """
    pass


class CallbackDisabledError(CallbackError):
    """
    Error for calling a deactivated Callback instance
    """
    pass


class CallbackAlreadyCalledError(CallbackError):
    """
    Error for calling a already called Callback instance
    """
    pass
