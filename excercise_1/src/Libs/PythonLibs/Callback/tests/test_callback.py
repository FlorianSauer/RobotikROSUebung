from unittest import TestCase

from Libs.PythonLibs.Callback import *


def test_method1(a, b, c='cdef', d='ddef'):
    # print a, b, c, d
    return a, b, c, d


class C(object):
    def test_method2(self, a, b, c='cdef', d='ddef'):
        # print self, a, b, c, d
        return self, a, b, c, d


class TestCallback(TestCase):

    def test_call_no_args(self):
        try:
            Callback(test_method1).call()
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)

    def test_call_one_args(self):
        try:
            Callback(test_method1).call('first')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_two_args_correct(self):
        self.assertTupleEqual(Callback(test_method1).call('first', 'second'), ('first', 'second', 'cdef', 'ddef'))

    def test___repr___two_args(self):
        self.assertEqual("<Callback test_method1(*, *)>", repr(Callback(test_method1)))

    def test_call_three_args_less(self):
        try:
            Callback(test_method1).call('first', 'second', 'third')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_one_args_one_kwargs_correct(self):
        self.assertTupleEqual(Callback(test_method1).call('first', b='second'), ('first', 'second', 'cdef', 'ddef'))

    def test_call_two_kwargs_correct(self):
        self.assertTupleEqual(Callback(test_method1).call(a='first', b='second'), ('first', 'second', 'cdef', 'ddef'))

    def test_call_one_arg_one_incorrect_kwarg(self):
        try:
            Callback(test_method1).call(a='first', e='wrong')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_one_arg_one_incorrect_kwarg_init(self):
        try:
            Callback(test_method1, kwargs={'a': 'first', 'e': 'wrong'})
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_two_arg_one_kwarg_correct(self):
        self.assertTupleEqual(Callback(test_method1).call('first', 'second', c='third'),
                              ('first', 'second', 'third', 'ddef'))

    def test_call_two_arg_two_kwarg_correct(self):
        self.assertTupleEqual(Callback(test_method1).call('first', 'second', c='third', d='fourth'),
                              ('first', 'second', 'third', 'fourth'))

    def test_call_second_default_arg_one_arg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('second',)).call('first'),
                              ('first', 'second', 'cdef', 'ddef'))

    def test___repr___second_default_arg_one_arg(self):
        self.assertEqual("<Callback test_method1(*, b='second')>", repr(Callback(test_method1, args=('second',))))

    def test_call_all_default_arg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('first', 'second')).call(),
                              ('first', 'second', 'cdef', 'ddef'))

    def test___repr___all_default_arg(self):
        self.assertEqual("<Callback test_method1(a='first', b='second')>",
                         repr(Callback(test_method1, args=('first', 'second'))))

    def test_call_three_default_arg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('first', 'second', 'third')).call(),
                              ('first', 'second', 'third', 'ddef'))

    def test___repr___three_default_arg(self):
        self.assertEqual("<Callback test_method1(a='first', b='second', c='third')>",
                         repr(Callback(test_method1, args=('first', 'second', 'third'))))

    def test_call_four_default_arg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('first', 'second', 'third', 'fourth')).call(),
                              ('first', 'second', 'third', 'fourth'))

    def test___repr___four_default_arg(self):
        self.assertEqual("<Callback test_method1(a='first', b='second', c='third', d='fourth')>",
                         repr(Callback(test_method1, args=('first', 'second', 'third', 'fourth'))))

    def test_call_second_default_arg_one_kwarg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('second',)).call(a='first'),
                              ('first', 'second', 'cdef', 'ddef'))

    def test_call_two_kwarg_correct(self):
        self.assertTupleEqual(Callback(test_method1, kwargs={'a': 'first', 'b': 'second'}).call(),
                              ('first', 'second', 'cdef', 'ddef'))

    def test___repr___two_kwarg(self):
        self.assertEqual("<Callback test_method1(a='first', b='second')>",
                         repr(Callback(test_method1, kwargs={'a': 'first', 'b': 'second'})))

    def test_call_two_kwarg_one_call_arg_correct(self):
        self.assertTupleEqual(Callback(test_method1, kwargs={'b': 'second', 'c': 'third'}).call('first'),
                              ('first', 'second', 'third', 'ddef'))

    def test___repr___two_kwarg_one_call_arg(self):
        self.assertEqual("<Callback test_method1(*, b='second', c='third')>",
                         repr(Callback(test_method1, kwargs={'b': 'second', 'c': 'third'})))

    def test_call_one_arg_one_kwarg_one_call_arg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('second',), kwargs={'c': 'third'}).call('first'),
                              ('first', 'second', 'third', 'ddef'))

    def test___repr___one_arg_one_kwarg_one_call_arg(self):
        self.assertEqual("<Callback test_method1(*, b='second', c='third')>",
                         repr(Callback(test_method1, args=('second',), kwargs={'c': 'third'})))

    def test_call_one_arg_one_kwarg_one_call_arg_one_call_kwarg_correct(self):
        self.assertTupleEqual(Callback(test_method1, args=('second',), kwargs={'c': 'third'}).call('first', d='fourth'),
                              ('first', 'second', 'third', 'fourth'))

    def test_call_three_kwarg_correct(self):
        self.assertTupleEqual(Callback(test_method1, kwargs={'a': 'first', 'b': 'second', 'c': 'third'}).call(),
                              ('first', 'second', 'third', 'ddef'))

    def test___repr___three_kwarg(self):
        self.assertEqual("<Callback test_method1(a='first', b='second', c='third')>",
                         repr(Callback(test_method1, kwargs={'a': 'first', 'b': 'second', 'c': 'third'})))

    def test_call_four_kwarg_correct(self):
        self.assertTupleEqual(
            Callback(test_method1, kwargs={'a': 'first', 'b': 'second', 'c': 'third', 'd': 'fourth'}).call(),
            ('first', 'second', 'third', 'fourth'))

    def test___repr___four_kwarg(self):
        self.assertEqual("<Callback test_method1(a='first', b='second', c='third', d='fourth')>", repr(
            Callback(test_method1, kwargs={'a': 'first', 'b': 'second', 'c': 'third', 'd': 'fourth'})))

    def test_call_multiple_values_for_kwarg(self):
        try:
            Callback(test_method1, args=('first', 'second', 'third_arg'), kwargs={'c': 'third_kwarg'})
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_replacing_second_arg_with_call_kwarg_correct(self):
        self.assertTupleEqual(
            Callback(test_method1, args=('second',), kwargs={'c': 'third'}).call('first', b='second_replaced'),
            ('first', 'second_replaced', 'third', 'ddef'))

    def test_object_call_no_args(self):
        obj = C()
        try:
            Callback(obj.test_method2).call()
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)

    def test_object_call_one_args(self):
        obj = C()
        try:
            Callback(obj.test_method2).call('first')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)
        else:
            self.fail("Expected exception did not occure")

    def test_object_call_two_args_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2).call('first', 'second'),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_three_args_less(self):
        obj = C()
        try:
            Callback(obj.test_method2).call('first', 'second', 'third')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)
        else:
            self.fail("Expected exception did not occure")

    def test_object_call_one_args_one_kwargs_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2).call('first', b='second'),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_two_kwargs_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2).call(a='first', b='second'),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_one_arg_one_incorrect_kwarg(self):
        obj = C()
        try:
            Callback(obj.test_method2).call(a='first', e='wrong')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_object_call_one_arg_one_incorrect_kwarg_init(self):
        obj = C()
        try:
            Callback(obj.test_method2, kwargs={'a': 'first', 'e': 'wrong'})
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_object_call_two_arg_one_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2).call('first', 'second', c='third'),
                              (obj, 'first', 'second', 'third', 'ddef'))

    def test_object_call_two_arg_two_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2).call('first', 'second', c='third', d='fourth'),
                              (obj, 'first', 'second', 'third', 'fourth'))

    def test_object_call_second_default_arg_one_arg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, args=('second',)).call('first'),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_all_default_arg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, args=('first', 'second')).call(),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_three_default_arg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, args=('first', 'second', 'third')).call(),
                              (obj, 'first', 'second', 'third', 'ddef'))

    def test_object_call_four_default_arg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, args=('first', 'second', 'third', 'fourth')).call(),
                              (obj, 'first', 'second', 'third', 'fourth'))

    def test_object_call_second_default_arg_one_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, args=('second',)).call('first'),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_two_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, kwargs={'a': 'first', 'b': 'second'}).call(),
                              (obj, 'first', 'second', 'cdef', 'ddef'))

    def test_object_call_two_kwarg_one_call_arg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, kwargs={'b': 'second', 'c': 'third'}).call('first'),
                              (obj, 'first', 'second', 'third', 'ddef'))

    def test_object_call_one_arg_one_kwarg_one_call_arg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, args=('second',), kwargs={'c': 'third'}).call('first'),
                              (obj, 'first', 'second', 'third', 'ddef'))

    def test_object_call_one_arg_one_kwarg_one_call_arg_one_call_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(
            Callback(obj.test_method2, args=('second',), kwargs={'c': 'third'}).call('first', d='fourth'),
            (obj, 'first', 'second', 'third', 'fourth'))

    def test_object_call_three_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(Callback(obj.test_method2, kwargs={'a': 'first', 'b': 'second', 'c': 'third'}).call(),
                              (obj, 'first', 'second', 'third', 'ddef'))

    def test_object_call_four_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(
            Callback(obj.test_method2, kwargs={'a': 'first', 'b': 'second', 'c': 'third', 'd': 'fourth'}).call(),
            (obj, 'first', 'second', 'third', 'fourth'))

    def test_object_call_multiple_values_for_kwarg(self):
        obj = C()
        try:
            Callback(obj.test_method2, args=('first', 'second', 'third_arg'), kwargs={'c': 'third_kwarg'})
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_object_call_replacing_second_arg_with_call_kwarg_correct(self):
        obj = C()
        self.assertTupleEqual(
            Callback(obj.test_method2, args=('second',), kwargs={'c': 'third'}).call('first', b='second_replaced'),
            (obj, 'first', 'second_replaced', 'third', 'ddef'))

    def test_call_lambda_noargs_correct(self):
        self.assertTupleEqual(Callback(lambda: test_method1('first', 'second')).call(),
                              ('first', 'second', 'cdef', 'ddef'))

    def test_call_lambda_one_arg_correct(self):
        self.assertTupleEqual(Callback(lambda x: test_method1(x, 'second')).call('first'),
                              ('first', 'second', 'cdef', 'ddef'))

    def test_call_lambda_two_arg_correct(self):
        self.assertTupleEqual(Callback(lambda x, y: test_method1(x, y)).call('first', 'second'),
                              ('first', 'second', 'cdef', 'ddef'))

    def test_call_lambda_too_much_args(self):
        c = Callback(lambda: test_method1('first', 'second'))
        try:
            c.call('error')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_lambda_method_kwarg_not_visible(self):
        c = Callback(lambda: test_method1('first', 'second'))
        try:
            c.call(a='error')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackKeywordArgumentError)
        else:
            self.fail("Expected exception did not occure")

    def test_call_lambda_method_kwarg_only_visible_for_lambda(self):
        self.assertTupleEqual(Callback(lambda x: test_method1(x, 'second')).call(x='first'),
                              ('first', 'second', 'cdef', 'ddef'))

    def test_call_lambda_too_much_args2(self):
        c = Callback(lambda x, y: test_method1(x, y))
        try:
            c.call('first', 'second', 'error')
        except CallbackError as e:
            self.assertExceptionEqual(e, CallbackArgumentCountError)
        else:
            self.fail("Expected exception did not occure")

    def assertExceptionEqual(self, exception, expected):
        try:
            self.assertTrue(issubclass(type(exception), Exception))
            self.assertEqual(type(exception), expected)
        except AssertionError:
            print repr(exception)
            raise
