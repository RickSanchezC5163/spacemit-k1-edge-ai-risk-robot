from setuptools import setup

package_name = "k1_light_control"

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
    description="GPIO37 light control node for the K1 inspection robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "adaptive_light_controller_node = k1_light_control.adaptive_light_controller_node:main",
            "gpio37_light_node = k1_light_control.gpio37_light_node:main",
            "pwm7_light_node = k1_light_control.pwm7_light_node:main",
            "risk_light_bridge_node = k1_light_control.risk_light_bridge_node:main",
        ],
    },
)
