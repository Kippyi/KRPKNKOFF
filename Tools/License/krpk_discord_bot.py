"""KRPKNKOFF license bot for Discord: testers get their own key with /activate, you manage keys with /license.

Runs on YOUR PC (next to the private signing key, which never goes to Discord). Testers can only get keys while it runs.

  python Tools/License/krpk_discord_bot.py --setup     one-time: bot token, server id, role names -> discord_bot.json
  python Tools/License/krpk_discord_bot.py             run the bot (or double-click Tools/License/Start_LicenseBot.bat)
  python Tools/License/krpk_discord_bot.py --selftest  offline test of the key rules (no Discord, temporary folder)

Commands (slash commands, all replies visible only to the person who typed them):
  /activate code:<machine code>     members with the tester role: a key for their PC (Discord name = watermark name).
                                    Same PC again = the same key back (or a renewal when it expires within renew_days).
                                    At most max_machines PCs per member; a PC registered to another member is refused.
  /mykeys                           your registered PCs, their keys and expiry dates
  /license issue name code days     admins: a key for anyone
  /license list                     admins: the last keys issued (from issued_keys.csv)
  /license block member | unblock member   admins: no new keys / renewals for that member
  /license check key                admins: decode and verify a key
  /license revoke member | key_id | machine | name    admins: withdraw keys (the game refuses them at its next start;
                                    needs `krpk_license.py revocation-setup` once); a member is also blocked
  /license unrevoke member | key_id | machine          admins: undo it

Files (in the signing folder, %USERPROFILE%\\KRPKNKOFF_Signing, never in the game folder):
  discord_bot.json      token + settings (the token is a password for the bot: never share it)
  discord_state.json    which member registered which PC (keys, expiry), blocked members
  issued_keys.csv       the shared ledger (also used by krpk_license.py)
"""
import argparse, asyncio, base64, datetime, getpass, json, os, re, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _lic():
    import krpk_license  # imported late so --selftest can point KRPK_SIGNING_DIR at a temporary folder first
    return krpk_license


DEFAULTS = {
    'token': '',
    'guild_id': 0,
    'tester_role': 'Tester',     # role name (or id) allowed to /activate
    'admin_role': 'Dev',         # role name (or id) allowed to /license ...; the server owner is always an admin
    'admin_user_ids': [],
    'days': 30,                  # key length for /activate
    'renew_days': 7,             # /activate on the same PC renews when the key expires within this many days
    'max_machines': 2,           # PCs per member
    'log_channel_id': 0,         # optional: a private channel where the bot posts who got a key (never the key itself)
}


def sanitize_name(display, user_id):
    """Discord display name -> watermark name (1-32 chars of A-Z a-z 0-9 space _ . -)."""
    s = re.sub(r'[^A-Za-z0-9 _.\-]', '', display or '')
    s = re.sub(r'\s+', ' ', s).strip()[:32].strip()
    return s if s else 'Tester-%s' % str(user_id)[-4:]


class LicenseDesk:
    """The key rules, independent of Discord (tested offline by --selftest)."""

    def __init__(self, priv, cfg, state_path):
        self.priv, self.cfg, self.state_path = priv, cfg, state_path
        self.state = {'users': {}, 'blocked': []}
        if os.path.exists(state_path):
            self.state = json.load(open(state_path, encoding='utf-8'))

    def _save(self):
        tmp = self.state_path + '.tmp'
        json.dump(self.state, open(tmp, 'w', encoding='utf-8'), indent=1)
        os.replace(tmp, self.state_path)

    def _owner_of(self, machine):
        for uid, u in self.state['users'].items():
            if machine in u['machines']:
                return uid
        return None

    def activate(self, user_id, display, machine, today=None):
        """Returns (ok, message, key or None)."""
        L = _lic()
        today = today or datetime.date.today()
        uid = str(user_id)
        machine = (machine or '').strip().upper()
        if uid in self.state['blocked']:
            return False, 'You cannot get keys at the moment. Ask the developer.', None
        if not L.MACHINE_RE.match(machine):
            return False, 'That does not look like a machine code. It looks like ABCD-EFGH-JKLM-NPQR (Copy button on the activation screen).', None
        owner = self._owner_of(machine)
        if owner is not None and owner != uid:
            return False, 'That PC is already registered to another member. Ask the developer if this is wrong.', None
        user = self.state['users'].setdefault(uid, {'name': sanitize_name(display, uid), 'machines': {}})
        rec = user['machines'].get(machine)
        if rec:
            exp = datetime.date.fromisoformat(rec['expires'])
            if (exp - today).days > self.cfg['renew_days']:
                return True, 'Your key for this PC (valid until %s):' % rec['expires'], rec['key']
        elif len(user['machines']) >= self.cfg['max_machines']:
            return False, 'You already have keys for %d PCs (the limit). Ask the developer to move one.' % len(user['machines']), None
        key, exp, key_id = L.issue_key(self.priv, user['name'], machine, self.cfg['days'], note='discord:%s' % uid)
        user['machines'][machine] = {'key': key, 'expires': exp.isoformat(), 'key_id': key_id}
        self._save()
        return True, ('Renewed. ' if rec else '') + 'Your key for PC %s, valid until %s. Paste it in the game and press Activate:' % (machine, exp.isoformat()), key

    def admin_issue(self, name, machine, days, by):
        key, exp, key_id = _lic().issue_key(self.priv, name, machine, days, note='discord-admin:%s' % by)
        return key, exp, key_id

    def my_keys(self, user_id):
        u = self.state['users'].get(str(user_id))
        return [] if not u else [(m, r['expires'], r['key']) for m, r in u['machines'].items()]

    def keys_of_member(self, user_id):
        """Every key id and PC of a member: from the bot's state and from ledger rows tagged discord:<id>."""
        L = _lic()
        uid = str(user_id)
        u = self.state['users'].get(uid, {'machines': {}})
        ids = {r['key_id'] for r in u['machines'].values()}
        machines = set(u['machines'].keys())
        if os.path.exists(L.LEDGER):
            import csv
            for r in csv.DictReader(open(L.LEDGER, encoding='utf-8')):
                if r.get('note', '') in ('discord:%s' % uid,):
                    ids.add(r['key_id'])
                    machines.add(r['machine'])
        return sorted(ids), sorted(machines)

    def revoke_member(self, user_id, undo=False):
        """Revokes (or restores) all of a member's keys and PCs, blocks (unblocks) them, publishes the list."""
        ids, machines = self.keys_of_member(user_id)
        self.block(user_id, not undo)
        _, _, status = _lic().revoke(self.priv, ids, machines, undo=undo)
        return ids, machines, status

    def revoke_items(self, key_ids=(), machines=(), names=(), undo=False):
        return _lic().revoke(self.priv, key_ids, machines, names, undo=undo)

    def block(self, user_id, on=True):
        uid = str(user_id)
        if on and uid not in self.state['blocked']:
            self.state['blocked'].append(uid)
        if not on and uid in self.state['blocked']:
            self.state['blocked'].remove(uid)
        self._save()

    def recent(self, n=15):
        L = _lic()
        if not os.path.exists(L.LEDGER):
            return []
        lines = open(L.LEDGER, encoding='utf-8').read().strip().splitlines()
        return lines[1:][-n:]

    def check(self, key):
        from cryptography.hazmat.primitives import hashes
        from cryptography.hazmat.primitives.asymmetric import padding
        L = _lic()
        head, _, sig = key.strip().replace('\n', '').replace(' ', '').partition('.')
        try:
            payload = L.b64u_dec(head)
            self.priv.public_key().verify(L.b64u_dec(sig), payload, padding.PKCS1v15(), hashes.SHA256())
            return True, payload.decode(errors='replace')
        except Exception:
            return False, 'not a valid key for this game'


# ---------------------------------------------------------------------------------------------------- Discord layer
def run_bot(cfg_path):
    import discord
    from discord import app_commands

    L = _lic()
    cfg = dict(DEFAULTS)
    cfg.update(json.load(open(cfg_path, encoding='utf-8')))
    if not cfg['token'] or not cfg['guild_id']:
        sys.exit('discord_bot.json needs token and guild_id: run with --setup.')
    desk = LicenseDesk(L.load_private(), cfg, os.path.join(L.SIGN_DIR, 'discord_state.json'))
    lock = asyncio.Lock()
    last_use = {}
    guild = discord.Object(id=int(cfg['guild_id']))
    client = discord.Client(intents=discord.Intents.default())
    tree = app_commands.CommandTree(client)

    def has_role(member, want):
        if not isinstance(member, discord.Member) or not want:
            return False
        return any(r.name == str(want) or str(r.id) == str(want) for r in member.roles)

    def is_admin(inter):
        m = inter.user
        return (inter.guild is not None and inter.guild.owner_id == m.id) or m.id in [int(x) for x in cfg['admin_user_ids']] or has_role(m, cfg['admin_role'])

    async def audit(text):
        if cfg['log_channel_id']:
            ch = client.get_channel(int(cfg['log_channel_id']))
            if ch:
                try:
                    await ch.send(text)
                except Exception:
                    pass

    @tree.command(name='activate', description='Get a KRPKNKOFF license key for your PC', guild=guild)
    @app_commands.describe(code='Machine code from the game\'s activation screen (ABCD-EFGH-JKLM-NPQR)')
    async def activate(inter: discord.Interaction, code: str):
        if not (has_role(inter.user, cfg['tester_role']) or is_admin(inter)):
            await inter.response.send_message('Only members with the %s role can get keys.' % cfg['tester_role'], ephemeral=True)
            return
        if time.time() - last_use.get(inter.user.id, 0) < 10:
            await inter.response.send_message('One moment, try again in a few seconds.', ephemeral=True)
            return
        last_use[inter.user.id] = time.time()
        async with lock:
            ok, msg, key = desk.activate(inter.user.id, inter.user.display_name, code)
        if key:
            await inter.response.send_message('%s\n```\n%s\n```' % (msg, key), ephemeral=True)
            await audit('Key for <@%d> (PC %s).' % (inter.user.id, code.strip().upper()))
        else:
            await inter.response.send_message(msg, ephemeral=True)

    @tree.command(name='mykeys', description='Your registered PCs and keys', guild=guild)
    async def mykeys(inter: discord.Interaction):
        rows = desk.my_keys(inter.user.id)
        if not rows:
            await inter.response.send_message('No keys yet. Use /activate with the machine code from the game.', ephemeral=True)
            return
        await inter.response.send_message('\n'.join('PC %s, valid until %s:\n```\n%s\n```' % r for r in rows), ephemeral=True)

    lic = app_commands.Group(name='license', description='KRPKNKOFF license admin', guild_ids=[int(cfg['guild_id'])])

    async def deny(inter):
        await inter.response.send_message('Admins only.', ephemeral=True)

    @lic.command(name='issue', description='Make a key for anyone')
    async def issue(inter: discord.Interaction, name: str, code: str, days: int = 30):
        if not is_admin(inter):
            return await deny(inter)
        try:
            async with lock:
                key, exp, key_id = desk.admin_issue(name, code, days, inter.user.id)
        except ValueError as e:
            return await inter.response.send_message(str(e), ephemeral=True)
        await inter.response.send_message('Key for %s (PC %s), valid until %s, id %s:\n```\n%s\n```' % (name, code.upper(), exp.isoformat(), key_id, key), ephemeral=True)
        await audit('Admin key for %s (PC %s) by <@%d>.' % (name, code.upper(), inter.user.id))

    @lic.command(name='list', description='Last keys issued')
    async def list_(inter: discord.Interaction):
        if not is_admin(inter):
            return await deny(inter)
        rows = desk.recent(15)
        await inter.response.send_message('```\nissued,expires,name,machine,key_id,note\n%s\n```' % '\n'.join(rows) if rows else 'No keys issued yet.', ephemeral=True)

    @lic.command(name='block', description='No new keys or renewals for a member')
    async def block(inter: discord.Interaction, member: discord.Member):
        if not is_admin(inter):
            return await deny(inter)
        desk.block(member.id, True)
        await inter.response.send_message('%s is blocked (their current keys still run until they expire).' % member.display_name, ephemeral=True)

    @lic.command(name='unblock', description='Allow a member to get keys again')
    async def unblock(inter: discord.Interaction, member: discord.Member):
        if not is_admin(inter):
            return await deny(inter)
        desk.block(member.id, False)
        await inter.response.send_message('%s can get keys again.' % member.display_name, ephemeral=True)

    @lic.command(name='check', description='Decode and verify a key')
    async def check(inter: discord.Interaction, key: str):
        if not is_admin(inter):
            return await deny(inter)
        ok, text = desk.check(key)
        await inter.response.send_message(('VALID: ' if ok else 'INVALID: ') + text, ephemeral=True)

    async def do_revoke(inter, member, key_id, machine, name, undo):
        if not is_admin(inter):
            return await deny(inter)
        if not (member or key_id or machine or name):
            return await inter.response.send_message('Give a member, key_id, machine or name.', ephemeral=True)
        await inter.response.defer(ephemeral=True, thinking=True)  # publishing to GitHub can take a few seconds
        try:
            async with lock:
                lines = []
                if member:
                    ids, machines, status = await asyncio.to_thread(desk.revoke_member, member.id, undo)
                    lines.append('%s %s: keys %s, PCs %s.' % ('Restored' if undo else 'Revoked', member.display_name, ', '.join(ids) or '-', ', '.join(machines) or '-'))
                if key_id or machine or name:
                    ids, machines, status = await asyncio.to_thread(desk.revoke_items, [key_id] if key_id else [], [machine] if machine else [],
                                                                    [name] if name and not undo else [], undo)
                    lines.append('%s keys %s, PCs %s.' % ('Restored' if undo else 'Revoked', ', '.join(ids) or '-', ', '.join(machines) or '-'))
        except Exception as e:
            return await inter.followup.send('Failed: %s' % e, ephemeral=True)
        await inter.followup.send('\n'.join(lines + [status]), ephemeral=True)
        await audit('%s by <@%d>: %s' % ('Restore' if undo else 'Revocation', inter.user.id, ' '.join(lines)))

    @lic.command(name='revoke', description='Revoke keys: a member (all their keys + PCs), or one key id / machine / name')
    @app_commands.describe(member='Revoke all keys and PCs of this member (and block them)', key_id='One key id (see /license list)',
                           machine='One machine code', name='All keys issued to this name')
    async def revoke(inter: discord.Interaction, member: discord.Member = None, key_id: str = None, machine: str = None, name: str = None):
        await do_revoke(inter, member, key_id, machine, name, False)

    @lic.command(name='unrevoke', description='Undo a revocation: a member, a key id or a machine')
    async def unrevoke(inter: discord.Interaction, member: discord.Member = None, key_id: str = None, machine: str = None):
        await do_revoke(inter, member, key_id, machine, None, True)

    tree.add_command(lic)

    @client.event
    async def on_ready():
        await tree.sync(guild=guild)
        print('License bot online as %s in server %s. Ctrl+C to stop.' % (client.user, cfg['guild_id']))

    client.run(cfg['token'])


def setup(cfg_path):
    L = _lic()
    cfg = dict(DEFAULTS)
    if os.path.exists(cfg_path):
        cfg.update(json.load(open(cfg_path, encoding='utf-8')))
    print('Discord license bot setup (Enter keeps the value in brackets).')
    tok = getpass.getpass('Bot token (hidden; Developer Portal > your app > Bot > Reset Token): ').strip()
    if tok:
        cfg['token'] = tok
    for k, q in [('guild_id', 'Server id (right-click the server icon > Copy Server ID)'), ('tester_role', 'Tester role name'),
                 ('admin_role', 'Admin role name'), ('days', 'Key length in days'), ('max_machines', 'PCs per member'),
                 ('log_channel_id', 'Private log channel id (0 = none)')]:
        v = input('%s [%s]: ' % (q, cfg[k])).strip()
        if v:
            cfg[k] = int(v) if isinstance(DEFAULTS[k], int) else v
    os.makedirs(L.SIGN_DIR, exist_ok=True)
    json.dump(cfg, open(cfg_path, 'w', encoding='utf-8'), indent=1)
    print('Saved %s. Start the bot with Tools\\License\\Start_LicenseBot.bat' % cfg_path)


def selftest():
    tmp = tempfile.mkdtemp(prefix='krpk_bot_test_')
    os.environ['KRPK_SIGNING_DIR'] = tmp
    L = _lic()
    from cryptography.hazmat.primitives.asymmetric import rsa
    priv = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    cfg = dict(DEFAULTS)
    desk = LicenseDesk(priv, cfg, os.path.join(tmp, 'state.json'))
    t0 = datetime.date(2026, 11, 20)
    A, B = 1001, 2002
    fails = []

    def expect(cond, what):
        print(('  ok   ' if cond else '  FAIL ') + what)
        if not cond:
            fails.append(what)

    ok, msg, k1 = desk.activate(A, 'Sam the Racer!!', 'abcd-efgh-jklm-npqr', today=t0)
    expect(ok and k1 and desk.check(k1)[0] and '|Sam the Racer|ABCD-EFGH-JKLM-NPQR|' in desk.check(k1)[1], 'tester gets a valid key; name sanitised; code upper-cased')
    ok, msg, k2 = desk.activate(A, 'Sam', 'ABCD-EFGH-JKLM-NPQR', today=t0)
    expect(ok and k2 == k1, 'same PC again -> the same key back')
    ok, msg, k3 = desk.activate(B, 'Alex', 'ABCD-EFGH-JKLM-NPQR', today=t0)
    expect(not ok and k3 is None, 'another member cannot take a registered PC')
    ok, msg, _ = desk.activate(A, 'Sam', 'not-a-code', today=t0)
    expect(not ok, 'garbage machine code refused')
    ok, _, _ = desk.activate(A, 'Sam', 'ZZZZ-ZZZZ-ZZZZ-2222', today=t0)
    ok2, msg, _ = desk.activate(A, 'Sam', 'YYYY-YYYY-YYYY-3333', today=t0)
    expect(ok and not ok2, 'limit of %d PCs per member' % cfg['max_machines'])
    exp = datetime.date.fromisoformat(desk.my_keys(A)[0][1])
    ok, msg, k4 = desk.activate(A, 'Sam', 'ABCD-EFGH-JKLM-NPQR', today=exp - datetime.timedelta(days=3))
    expect(ok and k4 != k1 and msg.startswith('Renewed'), 'near expiry the same PC gets a renewed key')
    desk.block(B)
    ok, msg, _ = desk.activate(B, 'Alex', 'QQQQ-QQQQ-QQQQ-4444', today=t0)
    expect(not ok, 'blocked member refused')
    desk.block(B, False)
    ok, msg, k5 = desk.activate(B, '', 'QQQQ-QQQQ-QQQQ-4444', today=t0)
    expect(ok and 'Tester-2002' in desk.check(k5)[1], 'unblocked; empty display name -> Tester-<id>')
    desk2 = LicenseDesk(priv, cfg, os.path.join(tmp, 'state.json'))
    expect(len(desk2.my_keys(A)) == 2, 'state survives a restart')
    expect(len(desk.recent(50)) == 4, 'every issued key is in the ledger (4: first, second PC, renewal, Alex)')
    try:
        desk.admin_issue('Bad|Name', 'ABCD-EFGH-JKLM-NPQR', 30, 1)
        expect(False, 'admin issue rejects a name with |')
    except ValueError:
        expect(True, 'admin issue rejects a name with |')
    expect(not desk.check(k1[:-4] + 'AAAA')[0], 'tampered key fails check')
    # Revocation
    ids, machines, status = desk.revoke_member(A)
    st = L.rev_load()
    expect(len(ids) == 3 and set(machines) == {'ABCD-EFGH-JKLM-NPQR', 'ZZZZ-ZZZZ-ZZZZ-2222'}, 'revoke member: all 3 of their keys (incl. the renewed one) + both PCs')
    expect(set(ids) <= set(st['key_ids']) and str(A) in desk.state['blocked'], 'revoked ids in the list; member blocked')
    text = open(L.REV_FILE).read().strip()
    head, _, sig = text.partition('.')
    payload = L.b64u_dec(head).decode()
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.asymmetric import padding
    try:
        priv.public_key().verify(L.b64u_dec(sig), payload.encode(), padding.PKCS1v15(), hashes.SHA256())
        sig_ok = True
    except Exception:
        sig_ok = False
    expect(sig_ok and payload.startswith('KRPKREV1|') and all(i in payload for i in ids), 'published list is signed and lists the keys')
    serial1 = st['serial']
    desk.revoke_member(A, undo=True)
    st = L.rev_load()
    expect(not st['key_ids'] and not st['machines'] and str(A) not in desk.state['blocked'] and st['serial'] > serial1, 'unrevoke member: list empty again, unblocked, newer serial')
    desk.revoke_items(names=['Alex'])
    expect(L.rev_load()['key_ids'] == [] , 'name with no ledger match revokes nothing (Alex got Tester-2002)')
    desk.revoke_items(names=['Tester-2002'])
    expect(len(L.rev_load()['key_ids']) == 1, 'revoke by watermark name')
    print('selftest: %d failure(s); temp folder %s' % (len(fails), tmp))
    return 1 if fails else 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--setup', action='store_true')
    p.add_argument('--selftest', action='store_true')
    a = p.parse_args()
    if a.selftest:
        sys.exit(selftest())
    cfg_path = os.path.join(_lic().SIGN_DIR, 'discord_bot.json')
    if a.setup:
        setup(cfg_path)
        return
    if not os.path.exists(cfg_path):
        sys.exit('No bot settings yet: run  python Tools\\License\\krpk_discord_bot.py --setup')
    run_bot(cfg_path)


if __name__ == '__main__':
    main()
