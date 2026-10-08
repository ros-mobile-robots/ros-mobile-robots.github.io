"""Write build/jobs.json: the render settings for assembly steps 16-22 and the 3D print page.

Positions are in mm in the base_link frame and come from holes.py; offsets ("explode", "adjust") are in metres.
"""
import json
from pathlib import Path

ASSEMBLY = "docs/hardware_setup/images/decks"
PRINT = "docs/hardware_setup/images/3d_print"
FRAME = r"base_link|rear_caster_.*|front_.*_wheel|.*_motor|power_bank_link|battery_pack_link|motor_driver_link|camera_mount_link"
# Corrections to the URDF placement: the breadboard sits on the deck plate, and the lidar platform's
# holes are 1.2 mm off the deck's holes in x.
ADJUST = {"breadboard_link": [0, 0, 0.002], "mcu_link": [0, 0, 0.002],
          "lidar_platform_link": [0.0012, 0, 0], "slamtec_holder_link": [0.0012, 0, 0],
          "slamtec_usb_to_serial_link": [0.0012, 0, 0], "rplidar_laser_link": [0.0012, 0, 0]}
FRONT, REAR, REAR_LEFT, CAMERA_VIEW = [1, -1.15, 1.0], [-1, -1.15, 1.05], [-1, 1.15, 1.05], [1.15, -1.0, 0.75]
INSERTS = [(-63, -44), (-63, 44), (69.5, -40.5), (69.5, 40.5)]     # brass inserts in the chassis posts
BOSSES = [(-4.1, -38.5), (-4.1, 19.5), (45, -38.5), (45, 19.5)]     # Raspberry Pi bosses on the deck
FEET = [(-55.4, -48.4), (-55.4, 48.4), (-23.7, -50.0), (-23.7, 50.0)]  # lidar platform feet on the deck
LIDAR = [(-58.9, -20.5), (-58.9, 21.2), (-17.2, -20.5), (-17.2, 21.2)]  # lidar holes in the platform
CAMERA = [(77.4, -31), (77.4, 31), (97.4, -31), (97.4, 31)]          # adapter holes on the camera mount


def show(*links):
    return "^(" + "|".join((FRAME,) + links) + ")$"


def screws(holes, z, lift):
    return [{"at": [x, y, z], "lift": lift, "len": 8, "d": 2.4} for x, y in holes]


SCENE = "/build/scene_rpi_raspi-cam.json"
DECK = ("rpi_deck_link",)
PI = DECK + ("rpi_link",)
BREADBOARD = PI + ("breadboard_link", "mcu_link")
PLATFORM = BREADBOARD + ("lidar_platform_link",)
LIDAR_ON = PLATFORM + ("rplidar_laser_link",)
HOLDER = LIDAR_ON + ("slamtec_holder_link", "slamtec_usb_to_serial_link")
steps = {
    "16-rpi-deck": dict(show=show(*DECK), explode={"rpi_deck_link": [0, 0, 0.04]}, highlight="rpi_deck_link",
                        screws=screws(INSERTS, 68.6, 58), noAutoGuide=True, view=FRONT),
    "17-raspberry-pi": dict(show=show(*PI), explode={"rpi_link": [0, 0, 0.035]},
                            screws=screws(BOSSES, 75.3, 55), noAutoGuide=True, view=FRONT),
    "18-breadboard": dict(show=show(*BREADBOARD), explode={"breadboard_link": [0, 0, 0.045], "mcu_link": [0, 0, 0.045]},
                          anchors={"breadboard_link": [[0.04, 0.04, 0], [0.96, 0.04, 0], [0.04, 0.96, 0], [0.96, 0.96, 0]],
                                   "mcu_link": []}, view=REAR),
    "19-lidar-platform": dict(show=show(*PLATFORM), explode={"lidar_platform_link": [0, 0, 0.045]},
                              highlight="lidar_platform_link", screws=screws(FEET, 72.5, 65), noAutoGuide=True, view=REAR),
    "20-lidar": dict(show=show(*LIDAR_ON), explode={"rplidar_laser_link": [0, 0, 0.04]}, noAutoGuide=True,
                     lines=[[[x, y, 148.8], [x, y, 108.8]] for x, y in LIDAR], view=REAR),
    "21-usb-adapter-holder": dict(show=show(*HOLDER),
                                  explode={"slamtec_holder_link": [0, 0, 0.05], "slamtec_usb_to_serial_link": [0, 0, 0.05]},
                                  highlight="slamtec_holder_link", anchors={"slamtec_holder_link": [[0.5, 0.5, 0]]},
                                  view=REAR_LEFT),
}
jobs = [{"out": f"{ASSEMBLY}/{name}.jpg", "cfg": {"scene": SCENE, **cfg}} for name, cfg in steps.items()]
for camera, lift in [("raspi-cam", 0.065), ("oak-1", 0.075), ("oak-d", 0.09)]:
    jobs.append({"out": f"{ASSEMBLY}/22-camera-{camera}.jpg", "cfg": {
        "scene": f"/build/scene_rpi_{camera}.json", "show": f"^(camera_mount_link|{camera}_link|camera_link)$",
        "explode": {f"{camera}_link": [0, 0, 0.03], "camera_link": [0, 0, lift]}, "highlight": f"{camera}_link",
        "screws": screws(CAMERA, 71.7, 42), "anchors": {f"{camera}_link": [], "camera_link": [[0.5, 0.5, 0]]},
        "view": CAMERA_VIEW}})
for job in jobs:
    job["cfg"]["adjust"] = ADJUST

for mesh in ["chassis", "caster/caster_base_65mm", "caster/caster_shroud_65mm", "deck/raspberry_pi_deck",
             "deck/jetson_nano_deck", "lidar_platform/platfom_rplidar_a2", "lidar_platform/platform_rplidar_a1",
             "lidar_platform/slamtec_holder", "camera_mount/camera_mount", "camera_mount/Raspberry_pi_CAM_holder",
             "camera_mount/OAK-1_adjustment_mount", "camera_mount/OAK-D_adjustment_mount"]:
    jobs.append({"out": f"{PRINT}/{mesh.split('/')[-1]}.jpg", "cfg": {
        "stl": f"meshes/remo/{mesh}.stl", "colors": {"part": 0x5c5c5c}, "view": [1, -1.2, 0.9],
        "width": 960, "height": 640, "margin": 0.06}})

out = Path(__file__).parent / "build" / "jobs.json"
out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(jobs, indent=1))
print(f"{len(jobs)} jobs in {out}")
