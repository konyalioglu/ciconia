# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 13:50:14 2022

@author: Turan Konyalioglu
"""

import numpy as np
from collections import defaultdict
import pickle
import main


def create_int_defaultdict():
    return defaultdict(dict)


with open('state_space_database.pickle', 'rb') as handle:
    data = pickle.load(handle)


