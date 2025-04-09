from utils import *
import os

SOFA_SRC=os.environ['SOFA_SRC']
filenames=[f"{SOFA_SRC}/applications/plugins/CMakeLists.txt",f"{SOFA_SRC}/applications/projects/CMakeLists.txt"]
presetFilename=f"{SOFA_SRC}/CMakePresets.json"
descriptionFolder=f"{SOFA_SRC}/scripts/subProjectLister/AdvancedDescriptions/"
presetFilname=f"{SOFA_SRC}/CMakePresets.json"

possibleTypes=["application","folder","plugin"]

allProjects = []
def saveProject(proj):
    allProjects.append(proj)

presetLookup = PresetLookup(presetFilname)




## Extract all subprojects
lines = []
for filename in filenames:
    cmakeFolderRelativePath = filename.removeprefix(f"{SOFA_SRC}/").removesuffix("CMakeLists.txt")
    with open(filename) as file:
        ## Transform into Projects type and sort them regarding type
        for line in [line.rstrip() for line in file]:
            if("sofa_add_subdirectory" in line):
                arguments = line.split('(')[1].split(')')[0]
                proj = InternalProjects.createFromArguments(arguments, cmakeFolderRelativePath, descriptionFolder)
                proj.extractPresets(presetLookup)
                if 'supported-plugins' in proj.presetList:
                    saveProject(proj)
            elif("sofa_add_external" in line):
                arguments = line.split('(')[1].split(')')[0]
                proj = ExternalProjects.createFromArguments(arguments,descriptionFolder)
                proj.extractPresets(presetLookup)
                if 'supported-plugins' in proj.presetList:
                    saveProject(proj)



## Sort list regarding the name of the project
allProjects = sortProjectByNames(allProjects)

## Print markdown tables
for project in allProjects:
    string = f" - {project.getDisplayName()}"
    if not project.isPluginized():
        string += "*"
    string += f": {project.getDescription()}"
    print(string)
print()
print("(*) Projects which sources are present in SOFA sources")