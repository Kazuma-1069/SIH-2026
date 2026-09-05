class NavigationState:
    """
    Stores autonomous navigation information.

    start:
        Initial vehicle position

    current:
        Current ego vehicle position

    destination:
        Final target position
    """

    def __init__(self):
        self.start = None
        self.current = None
        self.destination = None


    def set_start(self, location):
        self.start = location


    def update_current(self, location):
        self.current = location


    def set_destination(self, location):
        self.destination = location


    def get_planning_input(self):

        return {
            "ego_position": self.current,
            "goal": self.destination,
        }