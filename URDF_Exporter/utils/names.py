"""ASCII output aliases, separate from authoritative Fusion source identities."""
from collections import Counter
from hashlib import sha256
import re


_HELPER_PREFIXES = ('rigid_attachment_', 'fusion_helper_')


def sanitize_name(name):
    cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', str(name)).strip('_') or 'link'
    return 'link_' + cleaned if cleaned[0].isdigit() else cleaned


def allocate_names(identities, kind, *, root_identity=None):
    identities = tuple(identities)
    if len(set(identities)) != len(identities):
        raise ValueError('Duplicate %s identity: %s' % (kind, ', '.join(sorted(identities))))
    bases = {identity: sanitize_name(identity) for identity in identities if identity != root_identity}
    counts = Counter(base.casefold() for base in bases.values())
    aliases = {root_identity: 'base_link'} if root_identity is not None else {}
    pending = []
    for identity, base in bases.items():
        if counts[base.casefold()] == 1 and base.casefold() != 'base_link' and not base.casefold().startswith(_HELPER_PREFIXES):
            aliases[identity] = base
        else:
            pending.append(identity)
    used = {alias.casefold() for alias in aliases.values()} | {'base_link'}
    for identity in sorted(pending):
        base = bases[identity]
        if base.casefold().startswith(_HELPER_PREFIXES):
            base = 'source_' + base
        alias = base + '__' + sha256(identity.encode('utf-8')).hexdigest()[:12]
        while alias.casefold() in used:
            alias += '_'
        aliases[identity] = alias
        used.add(alias.casefold())
    return aliases


class ExportedJoints(dict):
    """Joint records carrying the allocation receipt, not an identity resolver."""

    def __init__(self, records, source_identifiers):
        super().__init__(records)
        self.source_identifiers = source_identifiers
