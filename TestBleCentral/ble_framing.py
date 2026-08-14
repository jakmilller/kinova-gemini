DEFAULT_CHUNK_BYTES = 180

def chunk_status(text: str, chunk_bytes: int = DEFAULT_CHUNK_BYTES):
    raw = text.encode("utf-8")
    if not raw:
        return[]

    payload_budget = max(1, chunk_bytes - 8)
    pieces = []
    i = 0
    while i < len(raw):
        piece = raw[i : i + payload_budget]

        for _ in range(3):
            try:
                piece.decode("utf-8")
                break
            except UnicodeDecodeError:
                piece = piece[:-1]

        if not piece:
            piece = raw[i : i + payload_budget]

        pieces.append(piece)
        i += len(piece)

    total = len(pieces)
    return [f"{n}/{total}:".encode("ascii") + p for n, p in enumerate(pieces, 1)]

def reassemble(frames):
    payloads = []
    for f in frames:
        header, _, payload = f.partition(b":")
        seq, _, total = header.partition(b"/")
        payloads.append((int(seq), payload))

    payloads.sort(key = lambda t: t[0])
    return b"".join(p for _, p in payloads).decode("utf-8")

def split_instructions(buf: bytearray):
    out = []
    while b"\n" in buf:
        line, _, rest = buf.partition(b"\n")
        buf = bytearray(rest)
        text = line.decode("utf-8", errors = "replace").strip()
        if text:
            out.append(text)

    return out, buf