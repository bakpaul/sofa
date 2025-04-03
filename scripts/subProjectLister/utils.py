import numpy as np

class Projects():

    name : str
    type : str
    presetList = list[str]
    advancedDescription = str

    def __init__(self, name, type):
        self.name = name
        self.type = type
        self.presetList = []
        self.advancedDescription = ""

    def getCorrectedName(self):
        return self.name.upper().replace('.','_')

    def extractAdvancedDescription(self,folderPath):
        #TODO
        self.advancedDescription = ""

    def extractPresets(self,presetFilename):
        #TODO
        self.presetList = []

    def getUpperType(self):
        return self.type.upper()

    def getDisplayName(self):
        pass

    def getDescription(self):
        pass

    def getActvation(self):
        pass

    @staticmethod
    def displayFirstRow():
        return f"|Name|Description|How to activate|\n|---|---|---|\n"

    def displayDescription(self):
        return f"|{self.getDisplayName()}|{self.getDescription()}|{self.getActvation()}|"


class InternalProjects(Projects):

    def __init__(self, name, type):
        Projects.__init__(self,name,type)

    @staticmethod
    def createFromArguments(stringArg : str):
        argList = stringArg.split()
        return InternalProjects(argList[1], argList[0])

    def getDisplayName(self):
        return f"{self.name}"

    def getDescription(self):
        return f"{self.type.capitalize()} named {self.name}"

    def getActvation(self):
        #TODO add presets
        return f"Set CMake flag '{self.getUpperType()}_{self.getCorrectedName()}' to ON"

class ExternalProjects(Projects):

    default_tag : str
    default_repo : str

    def __init__(self, name, type, default_tag,default_repo):
        Projects.__init__(self,name,type)
        self.default_tag = default_tag
        self.default_repo = default_repo

    @staticmethod
    def createFromArguments(stringArg : str):
        argList = stringArg.split()
        tagId = argList.index("GIT_REF") + 1
        repoId = argList.index("GIT_REPOSITORY") + 1
        return ExternalProjects(argList[1], argList[0], argList[tagId], argList[repoId])

    def getDisplayName(self):
        return f"[{self.name}]({self.default_repo})"

    def getDescription(self):
        return f"External {self.type.capitalize()} named {self.name} that needs ot be fetched"

    def getActvation(self):
        #TODO add presets
        return f"To fetch it set CMake flag 'SOFA_FETCH_{self.getCorrectedName()}' to ON, then activate it by setting CMake flag '{self.getUpperType()}_{self.getCorrectedName()}' to ON"


def printTableFromProjectListToString(projectList:Projects):
    output = Projects.displayFirstRow()
    for project in projectList:
        output += f"{project.displayDescription()}\n"
    return output

def sortProjectByNames(projectList:Projects):
    names = [proj.name for proj in projectList]
    sortedIdx = np.argsort(names).tolist()
    return [projectList[i] for i in sortedIdx]

