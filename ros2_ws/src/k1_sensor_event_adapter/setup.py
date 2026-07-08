from setuptools import setup

package_name = "k1_sensor_event_adapter"

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
    description="Real sensor event adapters for the non-arm K1 risk loop.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_event_adapter_node = k1_sensor_event_adapter.scan_event_adapter_node:main",
            "scan_safety_guard_node = k1_sensor_event_adapter.scan_safety_guard_node:main",
            "camera_low_light_adapter_node = k1_sensor_event_adapter.camera_low_light_adapter_node:main",
        ],
    },
)
