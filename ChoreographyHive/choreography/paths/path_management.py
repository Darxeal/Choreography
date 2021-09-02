import json
from dataclasses import dataclass
from pathlib import Path
from typing import List

from choreography.utils.vector_math import to_list
from rlutilities.linear_algebra import vec3

PATH_FOLDER = Path(__file__).parent / "saved"


@dataclass
class FormationPath:
    file: str
    points: List[vec3]
    formation_orientation: List[float]
    drone_orientation: List[float]
    params: List[List[float]]

    @staticmethod
    def load(file: str) -> 'FormationPath':
        with open(PATH_FOLDER / file, "r") as f:
            data = json.load(f)
            return FormationPath(
                file,
                [vec3(x, y, z) for x, y, z in data["points"]],
                data["formation_orientation"],
                data["drone_orientation"],
                data["params"]
            )

    def save(self):
        with open(PATH_FOLDER / self.file, "w") as f:
            json.dump({
                "points": [to_list(p) for p in self.points],
                "formation_orientation": self.formation_orientation,
                "drone_orientation": self.drone_orientation,
                "params": self.params,
            }, f, indent=4)
