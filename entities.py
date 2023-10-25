import json


class Query:
    operation_id: str
    operation_type: str
    source_dir: list
    output_dir: str
    params: dict

    def __init__(self, operation_id,
                 operation_type,
                 source_dir,
                 output_dir,
                 params):
        self.operation_id = operation_id
        self.operation_type = operation_type
        self.source_dir = source_dir
        self.output_dir = output_dir
        self.params = params

    def make_response(self):
        output_response = {
            "id": self.operation_id,
            "operation_type": self.operation_type,
            "file": self.output_dir
        }
        output_response = json.dumps(output_response)
        return output_response
