import time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean


class ModelDeleter:
    def __init__(self):
        self.node = Node()
        self.model_dict = {}

        print("Initializing ModelDeleter...")
        self.node.subscribe(
            topic="/world/beach_world/pose/info",
            msg_type=Pose_V,
            callback=self.update_model_dict
        )
        time.sleep(2)  # Allow time for initial data

    def update_model_dict(self, msg):
        """Update model dictionary with names and IDs."""
        self.model_dict = {pose.name: pose.id for pose in msg.pose}

    def delete_model(self, model_name):
        """Delete a model by name."""
        model_id = self.model_dict.get(model_name)
        if model_id is None:
            print(f"Model '{model_name}' not found.")
            return

        req = Entity(id=model_id, type=0)
        success, response = self.node.request(
            service="/world/beach_world/remove",
            request=req,
            request_type=Entity,
            response_type=Boolean,
            timeout=1000
        )

        if success and response.data:
            print(f"Successfully deleted model '{model_name}' (ID: {model_id}).")
            self.model_dict.pop(model_name, None)
        else:
            print(f"Failed to delete model '{model_name}' (ID: {model_id}).")

if __name__ == "__main__":
    deleter = ModelDeleter()
    while (model_name := input("Enter model name to delete or 'exit' to quit: ").strip().lower()) != "exit":
        deleter.delete_model(model_name)
    print("Goodbye!")