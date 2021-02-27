import json

class Parser:
    def __init__(self, fileName):
        self.data = None
        with open(fileName) as json_file:
            self.data = json.load(json_file)