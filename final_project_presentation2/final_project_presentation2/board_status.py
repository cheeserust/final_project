"""Parse the arm bridge's human-readable board status snapshot."""

from __future__ import annotations

import re
from typing import Any


def parse_arm_board_status(message: str) -> dict[str, Any]:
    """Return controllers and boards from ``/arm_board/status`` text."""
    raw = str(message or '')
    prefix = ''
    body = raw
    if ': ' in raw:
        prefix, body = raw.split(': ', 1)

    header = body.split('||', 1)[0]
    metadata: dict[str, Any] = {}
    for token in header.split(','):
        if '=' not in token:
            continue
        key, value = token.strip().split('=', 1)
        metadata[key.strip()] = _parse_scalar(value.strip())

    controllers: list[dict[str, Any]] = []
    boards: list[dict[str, Any]] = []
    pattern = r'([^\s\[\]|]+)\[(.*?)\](?=\s*(?:\|\||$))'
    for match in re.finditer(pattern, body):
        controller_name = match.group(1)
        controller_body = match.group(2)
        controller = {
            'name': controller_name,
            'accept_traj': None,
            'boards': [],
        }
        for piece in controller_body.split(';'):
            piece = piece.strip()
            if not piece:
                continue
            if piece.startswith('accept_traj='):
                controller['accept_traj'] = _parse_scalar(
                    piece.split('=', 1)[1].strip()
                )
                continue
            board = _parse_board_piece(piece)
            if board is None:
                continue
            board['controller'] = controller_name
            controller['boards'].append(board)
            boards.append(board)
        controllers.append(controller)

    return {
        'prefix': prefix,
        'arm_v3_state': metadata.get('arm_v3_state'),
        'active_goal': metadata.get('active_goal'),
        'controllers': controllers,
        'boards': boards,
        'raw': raw,
    }


def _parse_board_piece(piece: str) -> dict[str, Any] | None:
    match = re.match(r'board(\d+)(?::|=)\s*(.*)', piece)
    if match is None:
        return None
    fields: dict[str, Any] = {}
    notes: list[str] = []
    for token in match.group(2).split(','):
        token = token.strip()
        if not token:
            continue
        if '=' not in token:
            notes.append(token)
            continue
        key, value = token.split('=', 1)
        fields[key.strip()] = _parse_scalar(value.strip())
    return {
        'board_id': int(match.group(1)),
        'fields': fields,
        'notes': notes,
    }


def _parse_scalar(value: str) -> Any:
    if value == 'True':
        return True
    if value == 'False':
        return False
    if value in {'None', 'null'}:
        return None
    try:
        if value.lower().startswith('0x'):
            return int(value, 16)
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value
