from utils import *
import os

SOFA_SRC=os.environ['SOFA_SRC']
filenames=[f"{SOFA_SRC}/applications/plugins/CMakeLists.txt",f"{SOFA_SRC}/applications/projects/CMakeLists.txt"]
presetFilename=f"{SOFA_SRC}/CMakePresets.json"
descriptionFolder=f"{SOFA_SRC}/scripts/subProjectLister/AdvancedDescriptions/"
presetFilname=f"{SOFA_SRC}/CMakePresets.json"

possibleTypes=["application","folder","plugin"]

applications = []
folders = []
plugins = []

presetLookup = PresetLookup(presetFilname)

def saveProject(proj:Projects):
    global applications, folders, plugins
    match proj.type:
        case "application":
            applications.append(proj)
        case "directory":
            folders.append(proj)
        case "plugin":
            plugins.append(proj)


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
                saveProject(proj)
            elif("sofa_add_external" in line):
                arguments = line.split('(')[1].split(')')[0]
                proj = ExternalProjects.createFromArguments(arguments,descriptionFolder)
                proj.extractPresets(presetLookup)

                saveProject(proj)



## Sort list regarding the name of the project
applications = sortProjectByNames(applications)
plugins = sortProjectByNames(plugins)
folders = sortProjectByNames(folders)

## Print markdown tables



print("### Applications")
print()
print(printTableFromProjectListToString(applications))
print()

print("### Plugins")
print()
print(printTableFromProjectListToString(plugins))
print()

print("### Directories")
print()
print(printTableFromProjectListToString(folders))
print()
