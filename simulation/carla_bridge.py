import carla


class CarlaBridge:
    """Handles the connection between our project and the CARLA simulator."""

    def __init__(self, host="127.0.0.1", port=2010, timeout=10.0):
        self.host = host
        self.port = port
        self.timeout = timeout

        self.client = None
        self.world = None

    def connect(self):
        """Connect to CARLA and return the current world."""

        print(f"Connecting to CARLA at {self.host}:{self.port}...")

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)

        self.world = self.client.get_world()

        print("CARLA connected")
        print("Server:", self.client.get_server_version())
        print("Map:", self.world.get_map().name)

        return self.world

    def get_world(self):
        """Return the currently connected CARLA world."""
        return self.world

    def disconnect(self):
        """Release the CARLA connection."""
        self.client = None
        self.world = None
        print("CARLA disconnected")