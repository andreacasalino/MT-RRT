from random import random

def make_colors(n, default):
    if n <= len(default):
        return default[0:n]
    res = [col for col in default]
    while len(res) != n:
        res.append((random(), random(), random()))
    return res
