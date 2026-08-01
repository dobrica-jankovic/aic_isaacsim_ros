from setuptools import setup

package_name = "aic_insertion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/insertion.launch.py"]),
        (f"share/{package_name}/config", ["config/insertion.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Vision-based SFP plug insertion for the AIC UR5e scene.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = aic_insertion.perception_node:main",
            "insertion_node = aic_insertion.insertion_node:main",
        ],
    },
)
