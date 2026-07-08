from setuptools import setup

package_name = "k1_risk_engine"

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
    description="Rule-based risk engine for non-arm K1 bring-up.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "risk_engine_node = k1_risk_engine.risk_engine_node:main",
        ],
    },
)
