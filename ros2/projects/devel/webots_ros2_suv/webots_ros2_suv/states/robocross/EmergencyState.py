from webots_ros2_suv.states.AbstractState import AbstractState

class EmergencyState(AbstractState):
    # Реализация методов для StartState
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
            
    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

