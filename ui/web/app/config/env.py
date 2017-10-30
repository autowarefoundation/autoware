#!/usr/bin/env python
# coding: utf-8

import os
from dotenv import load_dotenv

load_dotenv(os.path.abspath(os.path.dirname(__file__))+'/.env')

env = dict(os.environ)
