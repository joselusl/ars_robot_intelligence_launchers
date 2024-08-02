from setuptools import find_packages, setup
import os

package_name = 'ars_robot_intelligence_launchers'


def list_subfolders_and_files(package_name, file_dir):
    result = []

    package_folder_share = os.path.join('share', package_name)
    root_folder = os.path.join(file_dir)
    
    def scan_folder(folder):
        files_in_folder = []
        subfolders = []

        # Iterate over each item in the folder
        for entry in os.scandir(folder):
            if entry.is_file():
                files_in_folder.append(entry.path)
            elif entry.is_dir():
                subfolders.append(entry.path)

        # Convert the absolute folder path to a relative path
        #relative_folder_path = os.path.relpath(folder, root_folder)

        # Exclude entries with empty file lists and entries for the root folder itself
        if files_in_folder:
            # Append a tuple of the relative folder path and the list of relative file paths to the result
            result.append((os.path.join(package_folder_share,folder), files_in_folder))

        # Recursively scan subfolders
        for subfolder in subfolders:
            scan_folder(subfolder)

    # Start scanning from the root folder
    scan_folder(root_folder)

    return result



data_files_list = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ]

config_files = list_subfolders_and_files(package_name, 'launch')


data_files_list += config_files



setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joselusl',
    maintainer_email='joseluis.sanlop@gmail.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
