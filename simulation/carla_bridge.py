import carla


class CarlaBridge:
    """Handles the connection between the SIH system and CARLA."""

    DEFAULT_HOST = "127.0.0.1"
    DEFAULT_PORT = 2010
    DEFAULT_TIMEOUT = 10.0

    def __init__(
        self,
        host=DEFAULT_HOST,
        port=DEFAULT_PORT,
        timeout=DEFAULT_TIMEOUT,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout

        self.client = None
        self.world = None

    def connect(self):
        """Connect to CARLA and return the current world."""

        if self.client is not None and self.world is not None:
            print("Already connected to CARLA.")
            return self.world

        print(f"Connecting to CARLA at {self.host}:{self.port}...")

        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)

            # Verify the CARLA server is reachable.
            server_version = self.client.get_server_version()

            # Get the currently loaded world.
            self.world = self.client.get_world()

            print("CARLA connected")
            print("Server:", server_version)
            print("Map:", self.world.get_map().name)

            return self.world

        except Exception as exc:
            self.client = None
            self.world = None

            print(f"CARLA connection failed: {exc}")
            raise ConnectionError(
                f"Could not connect to CARLA at "
                f"{self.host}:{self.port}"
            ) from exc

    def get_client(self):
        """Return the active CARLA client."""
        return self.client

    def get_world(self):
        """Return the currently connected CARLA world."""
        if self.world is None:
            raise RuntimeError(
                "CARLA world is not available. Call connect() first."
            )

        return self.world

    def is_connected(self):
        """Return True if a CARLA client and world are available."""
        return self.client is not None and self.world is not None

    def disconnect(self):
        """Release the local CARLA connection references."""

        self.client = None
        self.world = None

        print("CARLA disconnected")
                