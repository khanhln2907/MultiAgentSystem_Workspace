class AgentSimulationBase:
    def __init__(self) -> None:
        pass

    def move(self):
        raise NotImplementedError

    def getPose(self):
        raise NotImplementedError