import os
import sys
import ast
import inspect
import time
import logging
import code
import jedi
import cStringIO
import inspect
import signal
import thread
import gevent
from contextlib import contextmanager
from bliss.common.event import dispatcher
from bliss.common import data_manager
from bliss.config import static as beacon_static
jedi.settings.case_insensitive_completion = False

class Stdout:

    def __init__(self, queue):
        self.queue = queue

    def flush(self):
        pass

    def write(self, output):
        self.queue.put(output)        


@contextmanager
def stdout_redirected(new_stdout):
    save_stdout = sys.stdout
    sys.stdout = new_stdout
    try:
        yield None
    finally:
        sys.stdout = save_stdout


def init_scans_callbacks(output_queue):
    def new_scan_callback(scan_id, filename, scan_actuators, npoints, counters_list):
        output_queue.put({"scan_id": scan_id, "filename": filename,
                          "scan_actuators": scan_actuators, "npoints": npoints,
                          "counters": counters_list})
    def update_scan_callback(scan_id, values):
        output_queue.put({"scan_id": scan_id, "values":values})
    def scan_end_callback(scan_id):
        output_queue.put({"scan_id":scan_id})

    # keep callbacks references
    output_queue.new_scan_callback = new_scan_callback
    output_queue.update_scan_callback = update_scan_callback
    output_queue.scan_end_callback = scan_end_callback

    dispatcher.connect(
        new_scan_callback, "scan_new", data_manager.DataManager())
    dispatcher.connect(
        update_scan_callback, "scan_data", data_manager.DataManager())
    dispatcher.connect(
        scan_end_callback, "scan_end", data_manager.DataManager())


class InteractiveInterpreter(code.InteractiveInterpreter):

    def __init__(self, output_queue):
        code.InteractiveInterpreter.__init__(self) #, globals_dict)

        self.error = cStringIO.StringIO()
        self.output = Stdout(output_queue)
        self.executed_greenlet = None

    def write(self, data):
        self.error.write(data)

    def kill(self, exception):
        if self.executed_greenlet and not self.executed_greenlet.ready():
            self.executed_greenlet.kill(exception)
            return True
        return False

    def runcode(self, c):
        try:
            with stdout_redirected(self.output):
                exec c in self.locals
        except SystemExit:
            raise
        except KeyboardInterrupt:
            self.error.write("KeyboardInterrupt")
        except:
            self.showtraceback()

    def compile_and_run(self, python_code_to_execute):
        code_obj = None

        try:
            code_obj = code.compile_command(python_code_to_execute)
        except SyntaxError, exc_instance:
            raise RuntimeError(str(exc_instance))
        else:
            if code_obj is None:
                # input is incomplete
                raise EOFError
            else:
                self.runcode(code_obj)
                
                if self.error.tell() > 0:
                    error_string = self.error.getvalue()
                    self.error = cStringIO.StringIO()
                    raise RuntimeError(error_string)

    def execute(self, python_code_to_execute):
        self.executed_greenlet = gevent.spawn(self.compile_and_run, python_code_to_execute)
        return self.executed_greenlet.get()


def init(input_queue, output_queue):
    # undo thread module monkey-patching
    reload(thread)

    i = InteractiveInterpreter(output_queue)

    return i

def start(input_queue, output_queue, i):
    # restore default SIGINT behaviour
    def raise_kb_interrupt(interpreter=i):
        if not interpreter.kill(KeyboardInterrupt):
            raise KeyboardInterrupt
    gevent.signal(signal.SIGINT, raise_kb_interrupt)

    init_scans_callbacks(output_queue)
    output_queue.motor_callbacks = dict()

    def resetup(setup_file=None):
        setup_file = i.locals.get("SETUP_FILE") if setup_file is None else setup_file
        if setup_file is not None:
            i.locals["SETUP_FILE"] = setup_file
            setup_file_path = os.path.abspath(os.path.expanduser(setup_file))
            if os.path.isfile(setup_file_path):
                setup_file_dir = os.path.dirname(setup_file_path)
                if not setup_file_dir in sys.path:
                    sys.path.insert(0, setup_file_dir)
                execfile(setup_file_path, i.locals)
    i.locals["resetup"] = resetup

    while True:
        action, _ = input_queue.get()
        if action == "syn":
            output_queue.put("ack")
            continue
        elif action == "motors_list":
            motors_list = []
           
            for name, x in i.locals.iteritems():
                if name in beacon_static.MOTORS:
                    pos = "%.3f" % x.position()
                    motors_list.append({ "name": x.name, "state": x.state(), "pos": pos })
                    def state_updated(state, name=x.name):
                        output_queue.put({"name":name, "state": state})
                    def position_updated(pos, name=x.name):
                        pos = "%.3f" % pos
                        output_queue.put({"name":name, "position":pos})
                    output_queue.motor_callbacks[x.name]=(state_updated, position_updated) 
                    dispatcher.connect(state_updated, "state", x)
                    dispatcher.connect(position_updated, "position", x)
            motors_list = sorted(motors_list, cmp=lambda x,y: cmp(x["name"],y["name"]))
            print motors_list
            output_queue.put(StopIteration(motors_list))
        elif action == "execute":
            code = _[0]
            try:
                i.execute(code)
            except EOFError:
                output_queue.put(StopIteration(EOFError()))
            except RuntimeError, error_string:
                print error_string
                output_queue.put(StopIteration(RuntimeError(error_string)))
            else:           
                output_queue.put(StopIteration(None))
        elif action == "complete":
            text, completion_start_index = _
            completion_obj = jedi.Interpreter(text, [i.locals], line=1, column=completion_start_index)
            possibilities = []
            completions = []
            for x in completion_obj.completions():
                possibilities.append(x.name)
                completions.append(x.complete)
            output_queue.put(StopIteration((possibilities, completions)))
        elif action == "get_function_args":
            code = _[0]
            try:
                ast_node = ast.parse(code)
            except:
                output_queue.put(StopIteration({"func": False}))
            else:
                if isinstance(ast_node.body[-1], ast.Expr):
                    expr = code[ast_node.body[-1].col_offset:]
                    try:
                        x = eval(expr, i.locals)
                    except:
                        output_queue.put(StopIteration({"func": False}))
                    else:
                        if callable(x):
                            try:
                                x.__call__
                            except AttributeError:
                                if inspect.isfunction(x):
                                    args = inspect.formatargspec(*inspect.getargspec(x))
                                elif inspect.ismethod(x):
                                    argspec = inspect.getargspec(x)
                                    args = inspect.formatargspec(argspec.args[1:],*argspec[1:])
                                else:
                                    output_queue.put(StopIteration({"func": False}))
                                    continue
                            else:
                                output_queue.put(StopIteration({"func": False}))
                                continue
                                # like a method
                                #argspec = inspect.getargspec(x.__call__)
                                #args = inspect.formatargspec(argspec.args[1:],*argspec[1:])
                            #print {"func": True, "func_name":expr, "args": args }
                            output_queue.put(StopIteration({"func": True, "func_name":expr, "args": args }))
                        else:
                            output_queue.put(StopIteration({"func": False}))

