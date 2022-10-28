import json
from datetime import datetime

class EventMessage:
    def __init__(self, event_type, metadata):
        self.event_type = event_type
        self.timestamp = datetime.timestamp(datetime.now())
        self.metadata = metadata

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)
