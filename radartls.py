#!/usr/bin/env python3
"""
radartls.py - makes the https certificate radarweb serves on port 8443 (self-signed: browsers warn once, you tap
"Advanced" -> "Proceed"). Run on the EON:

  python3 /data/claude-stuff/radartls.py 192.168.0.252            certificate for that address (add more: IPs or names)
  python3 /data/claude-stuff/radartls.py 192.168.0.252 192.168.43.1 myeon.duckdns.org

Writes /data/radarweb_tls/cert.pem and key.pem (key readable by root only), then restart radarweb (reboot) and open
https://192.168.0.252:8443/. A certificate only matches the addresses listed here: if the EON gets another IP
(other Wi-Fi, hotspot), list that one too and run this again.

A trusted certificate instead (no warning) needs a public name pointing at the EON, e.g. Let's Encrypt for a
DuckDNS name: put its fullchain as cert.pem and its private key as key.pem in the same folder.
"""
import datetime
import ipaddress
import os
import subprocess
import sys
import tempfile

OUT = os.environ.get('RADARWEB_TLS_DIR', '/data/radarweb_tls')
DAYS = 825


def split(names):
    ips, dns = [], []
    for n in names:
        try: ips.append(ipaddress.ip_address(n))
        except ValueError: dns.append(n)
    return ips, dns


def with_cryptography(names, cert_path, key_path):
    from cryptography import x509
    from cryptography.hazmat.backends import default_backend
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import NameOID
    ips, dns = split(names)
    key = ec.generate_private_key(ec.SECP256R1(), default_backend())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, u'radarweb ' + names[0])])
    now = datetime.datetime.utcnow()
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=DAYS))
            .add_extension(x509.SubjectAlternativeName([x509.IPAddress(i) for i in ips] + [x509.DNSName(d) for d in dns]), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.ExtendedKeyUsage([x509.oid.ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .sign(key, hashes.SHA256(), default_backend()))
    write(key_path, key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
                                      serialization.NoEncryption()), 0o600)
    write(cert_path, cert.public_bytes(serialization.Encoding.PEM), 0o644)


def with_openssl(names, cert_path, key_path):
    ips, dns = split(names)
    san = ','.join(['IP:%s' % i for i in ips] + ['DNS:%s' % d for d in dns])
    with tempfile.NamedTemporaryFile('w', suffix='.cnf', delete=False) as f:
        f.write('[req]\ndistinguished_name=dn\nx509_extensions=ext\nprompt=no\n[dn]\nCN=radarweb %s\n'
                '[ext]\nsubjectAltName=%s\nbasicConstraints=critical,CA:FALSE\nextendedKeyUsage=serverAuth\n' % (names[0], san))
        cnf = f.name
    try:
        subprocess.check_call(['openssl', 'req', '-x509', '-newkey', 'ec', '-pkeyopt', 'ec_paramgen_curve:prime256v1',
                               '-nodes', '-days', str(DAYS), '-keyout', key_path, '-out', cert_path, '-config', cnf])
        os.chmod(key_path, 0o600)
    finally:
        os.remove(cnf)


def write(path, data, mode):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    with os.fdopen(fd, 'wb') as f:
        f.write(data)
    os.chmod(path, mode)


def main():
    names = sys.argv[1:]
    if not names or names[0] in ('-h', '--help'):
        sys.exit(__doc__)
    os.makedirs(OUT, exist_ok=True)
    cert_path, key_path = os.path.join(OUT, 'cert.pem'), os.path.join(OUT, 'key.pem')
    try:
        with_cryptography(names, cert_path, key_path)
    except ImportError:
        with_openssl(names, cert_path, key_path)
    print('wrote %s and %s for %s (valid %d days)' % (cert_path, key_path, ', '.join(names), DAYS))
    print('restart radarweb (reboot), then open https://%s:8443/ and accept the warning once' % names[0])


if __name__ == '__main__':
    main()
