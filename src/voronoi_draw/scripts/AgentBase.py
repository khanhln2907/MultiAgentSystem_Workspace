class AgentBase:
    ID = -1


    def __init__(self, ID) -> None:
        self.ID = ID
        pass

    def move(self):
        raise NotImplementedError

    def getPose(self):
        raise NotImplementedError