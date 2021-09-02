from dataclasses import dataclass
from typing import List


@dataclass
class ChoreographyGroup:
    indexes: List[int]
    teams: List[int]
    appearances: List[str]

    def __add__(self, other: 'ChoreographyGroup') -> 'ChoreographyGroup':
        return ChoreographyGroup(
            self.indexes + other.indexes,
            self.teams + other.teams,
            self.appearances + other.appearances
        )


class ChoreographyGroupFactory:
    def __init__(self) -> None:
        self.last_index = 0

    def make_group(self, count: int, team: int, appearance: str) -> ChoreographyGroup:
        group = ChoreographyGroup(
            indexes=list(range(self.last_index, self.last_index + count)),
            teams=[team] * count,
            appearances=[appearance] * count
        )
        self.last_index += count
        return group
