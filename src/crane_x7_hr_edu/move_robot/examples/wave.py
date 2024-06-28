import sys

sys.path.append("..")
from topic_publisher import angle_write

wave = [
    [90,0,0,0,0,0,0,60],
    [90,20,0,0,0,0,0,60],
    [90,-20,0,0,0,0,0,60],
    [90,20,0,0,0,0,0,60],
    [90,-20,0,0,0,0,0,60],
    [90,0,0,0,0,0,0,60]
]

for value in wave:
    angle_write(value)