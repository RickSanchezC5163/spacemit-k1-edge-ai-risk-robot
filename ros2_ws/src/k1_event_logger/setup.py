from setuptools import setup

package_name = "k1_event_logger"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="K1 Robot Team",
    maintainer_email="team@example.invalid",
    description="JSONL event logger for K1 non-arm bring-up tests.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "event_logger_node = k1_event_logger.event_logger_node:main",
        ],
    },
)
