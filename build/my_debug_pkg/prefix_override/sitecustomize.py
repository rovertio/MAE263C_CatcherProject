import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/manuel/MAE263C_CatcherProject/install/my_debug_pkg'
