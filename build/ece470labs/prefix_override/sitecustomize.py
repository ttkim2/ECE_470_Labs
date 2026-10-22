import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aaronjl4/Desktop/ECE_470_Labs/install/ece470labs'
