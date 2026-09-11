#define _CRT_SECURE_NO_WARNINGS

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIDI_BYTE_GAP_DEFAULT 7680ULL
/* Longest event kept whole in the schedule; SysEx (GS/GM reset, dumps) must
 * survive the conversion or the playback patches/drum kits are wrong. */
#define MAX_EVENT_BYTES 1024

typedef struct {
    unsigned long long tick;
    unsigned long long cycle;
    int track;
    int seq;
    unsigned char bytes[MAX_EVENT_BYTES];
    int nbytes;
    int sysex; /* all bytes posted at one cycle (no byte_gap) */
} Event;

typedef struct {
    unsigned long long tick;
    unsigned long long cycle;
    unsigned long tempo_us;
} Tempo;

typedef struct {
    unsigned long long cycle;
    int seq;
    int byte_idx;
    unsigned char byte;
} OutLine;

typedef struct {
    unsigned char *buf;
    size_t size;
    size_t pos;
} Reader;

static void usage(FILE *out, const char *prog)
{
    fprintf(out,
            "usage: %s <in.mid> [-o out.sched] [--start cycles] [--ppq N]\n"
            "          [--byte-gap cycles] [--verbose]\n"
            "\n"
            "Emits '<cycle> <hexbyte> [<hexbyte>...]' lines (schedule cycles are\n"
            "relative to the GT -midiseq <file> [start] argument).\n",
            prog);
}

static void die(const char *msg, const char *arg)
{
    if (arg)
        fprintf(stderr, "midisched: %s '%s'\n", msg, arg);
    else
        fprintf(stderr, "midisched: %s\n", msg);
    exit(2);
}

static unsigned char *read_file(const char *path, size_t *out_len)
{
    FILE *f;
    long len;
    unsigned char *buf;
    size_t got;

    f = fopen(path, "rb");
    if (!f)
        die("cannot open input", path);
    if (fseek(f, 0, SEEK_END) != 0)
        die("cannot seek input", path);
    len = ftell(f);
    if (len < 0)
        die("cannot tell input", path);
    if (fseek(f, 0, SEEK_SET) != 0)
        die("cannot rewind input", path);
    buf = (unsigned char *)malloc((size_t)len + 1);
    if (!buf)
        die("out of memory", NULL);
    got = fread(buf, 1, (size_t)len, f);
    if (got != (size_t)len)
        die("short read on input", path);
    fclose(f);
    *out_len = (size_t)len;
    return buf;
}

static int r_u8(Reader *r, unsigned *out)
{
    if (r->pos + 1 > r->size)
        return 0;
    *out = r->buf[r->pos++];
    return 1;
}

static int r_be16(Reader *r, unsigned *out)
{
    if (r->pos + 2 > r->size)
        return 0;
    *out = ((unsigned)r->buf[r->pos] << 8) | r->buf[r->pos + 1];
    r->pos += 2;
    return 1;
}

static int r_be32(Reader *r, unsigned long *out)
{
    if (r->pos + 4 > r->size)
        return 0;
    *out = ((unsigned long)r->buf[r->pos] << 24) |
           ((unsigned long)r->buf[r->pos + 1] << 16) |
           ((unsigned long)r->buf[r->pos + 2] << 8) |
           (unsigned long)r->buf[r->pos + 3];
    r->pos += 4;
    return 1;
}

static int r_vlq(Reader *r, unsigned long *out)
{
    unsigned long v = 0;
    unsigned char b;
    int i;
    for (i = 0; i < 4; i++) {
        if (r->pos >= r->size)
            return 0;
        b = r->buf[r->pos++];
        v = (v << 7) | (unsigned long)(b & 0x7F);
        if (!(b & 0x80)) {
            *out = v;
            return 1;
        }
    }
    return 0;
}

static int parse_args(int argc, char **argv, const char **in_path,
                      const char **out_path, unsigned long long *start,
                      unsigned long *ppq_override, unsigned long long *byte_gap,
                      int *verbose)
{
    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            *out_path = argv[++i];
        } else if (strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
            char *end;
            const char *v = argv[++i];
            errno = 0;
            *start = strtoull(v, &end, 10);
            if (end == v || *end || errno == ERANGE) {
                fprintf(stderr, "midisched: invalid --start value '%s'\n", v);
                return 0;
            }
        } else if (strcmp(argv[i], "--ppq") == 0 && i + 1 < argc) {
            char *end;
            const char *v = argv[++i];
            unsigned long n;
            errno = 0;
            n = strtoul(v, &end, 10);
            if (end == v || *end || errno == ERANGE || n == 0 || n > 0x7FFF) {
                fprintf(stderr, "midisched: invalid --ppq value '%s'\n", v);
                return 0;
            }
            *ppq_override = n;
        } else if (strcmp(argv[i], "--byte-gap") == 0 && i + 1 < argc) {
            char *end;
            const char *v = argv[++i];
            errno = 0;
            *byte_gap = strtoull(v, &end, 10);
            if (end == v || *end || errno == ERANGE) {
                fprintf(stderr, "midisched: invalid --byte-gap value '%s'\n", v);
                return 0;
            }
        } else if (strcmp(argv[i], "--verbose") == 0) {
            *verbose = 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(stdout, argv[0]);
            exit(0);
        } else if (argv[i][0] == '-' && argv[i][1] != '\0') {
            fprintf(stderr, "midisched: unknown option '%s'\n", argv[i]);
            return 0;
        } else if (!*in_path) {
            *in_path = argv[i];
        } else {
            fprintf(stderr, "midisched: unexpected argument '%s'\n", argv[i]);
            return 0;
        }
    }
    if (!*in_path) {
        usage(stderr, argv[0]);
        return 0;
    }
    return 1;
}

static int cmp_event(const void *a, const void *b)
{
    const Event *ea = (const Event *)a;
    const Event *eb = (const Event *)b;
    if (ea->cycle != eb->cycle)
        return ea->cycle < eb->cycle ? -1 : 1;
    if (ea->track != eb->track)
        return ea->track < eb->track ? -1 : 1;
    if (ea->seq != eb->seq)
        return ea->seq < eb->seq ? -1 : 1;
    return 0;
}

static int cmp_tempo(const void *a, const void *b)
{
    const Tempo *ta = (const Tempo *)a;
    const Tempo *tb = (const Tempo *)b;
    if (ta->tick != tb->tick)
        return ta->tick < tb->tick ? -1 : 1;
    return 0;
}

static int cmp_outline(const void *a, const void *b)
{
    const OutLine *la = (const OutLine *)a;
    const OutLine *lb = (const OutLine *)b;
    if (la->cycle != lb->cycle)
        return la->cycle < lb->cycle ? -1 : 1;
    if (la->seq != lb->seq)
        return la->seq < lb->seq ? -1 : 1;
    return la->byte_idx - lb->byte_idx;
}

static void push_byte(OutLine **lines, size_t *n, size_t *cap,
                      unsigned long long cycle, int seq, int byte_idx,
                      unsigned char byte)
{
    if (*n == *cap) {
        size_t ncap = *cap ? *cap * 2 : 1024;
        OutLine *nl = (OutLine *)realloc(*lines, ncap * sizeof(OutLine));
        if (!nl)
            die("out of memory", NULL);
        *lines = nl;
        *cap = ncap;
    }
    (*lines)[*n].cycle = cycle;
    (*lines)[*n].seq = seq;
    (*lines)[*n].byte_idx = byte_idx;
    (*lines)[*n].byte = byte;
    (*n)++;
}

int main(int argc, char **argv)
{
    const char *in_path = NULL;
    const char *out_path = NULL;
    unsigned long long start = 0;
    unsigned long ppq_override = 0;
    unsigned long long byte_gap = MIDI_BYTE_GAP_DEFAULT;
    int verbose = 0;
    unsigned char *buf;
    size_t size;
    Reader r;
    unsigned long hdr_len = 0;
    unsigned format = 0, ntrks = 0, division = 0;
    int ppq;
    int smpte = 0;
    unsigned fps = 0, tpf = 0;
    Event *events = NULL;
    size_t nevents = 0, events_cap = 0;
    Tempo *tempos = NULL;
    size_t ntempos = 0, tempos_cap = 0;
    unsigned long long *track_tick = NULL;
    int t;
    OutLine *lines = NULL;
    size_t nlines = 0, lines_cap = 0;
    FILE *out;
    size_t i;

    if (!parse_args(argc, argv, &in_path, &out_path, &start, &ppq_override,
                    &byte_gap, &verbose))
        return 2;

    buf = read_file(in_path, &size);
    r.buf = buf;
    r.size = size;
    r.pos = 0;
    {
        unsigned char id[4];
        if (size < 8 || memcmp(buf, "MThd", 4) != 0)
            die("not a Standard MIDI File", in_path);
        r.pos = 4;
        if (!r_be32(&r, &hdr_len) || hdr_len < 6)
            die("bad MThd header", in_path);
        if (!r_be16(&r, &format) || !r_be16(&r, &ntrks) || !r_be16(&r, &division))
            die("truncated MThd header", in_path);
        (void)id;
        r.pos = 8 + hdr_len;
    }
    if (format > 1)
        die("only SMF format 0/1 is supported (format 2 is sequential)", in_path);
    if (ntrks == 0 || ntrks > 256)
        die("unsupported track count", in_path);

    if (ppq_override) {
        ppq = (int)ppq_override;
    } else if (division & 0x8000) {
        smpte = 1;
        {
            int fps_i = -(int)((division >> 8) & 0xFF);
            fps = (unsigned)(fps_i < 0 ? -fps_i : fps_i);
            tpf = division & 0xFF;
        }
        if (fps == 0 || tpf == 0)
            die("bad SMPTE division", in_path);
        ppq = 0;
    } else {
        ppq = (int)division;
        if (ppq == 0)
            die("bad PPQ division", in_path);
    }

    track_tick = (unsigned long long *)calloc(ntrks, sizeof(unsigned long long));
    if (!track_tick)
        die("out of memory", NULL);

    for (t = 0; t < (int)ntrks; t++) {
        unsigned long trk_len;
        unsigned char magic[4];
        int seq = 0;
        unsigned char running = 0;

        if (r.pos + 8 > r.size)
            break;
        memcpy(magic, r.buf + r.pos, 4);
        r.pos += 4;
        if (!r_be32(&r, &trk_len))
            die("truncated track header", in_path);
        if (memcmp(magic, "MTrk", 4) != 0)
            die("expected MTrk chunk", in_path);
        if (r.pos + trk_len > r.size)
            die("truncated track data", in_path);
        {
            Reader tr;
            tr.buf = r.buf;
            tr.size = r.pos + trk_len;
            tr.pos = r.pos;
            while (tr.pos < tr.size) {
                unsigned long delta;
                unsigned status, b0;
                if (!r_vlq(&tr, &delta))
                    die("bad delta time", in_path);
                track_tick[t] += delta;
                if (!r_u8(&tr, &status))
                    die("truncated event", in_path);
                if (status < 0x80) {
                    if (!running)
                        die("running status without a channel status", in_path);
                    b0 = status;
                    status = running;
                } else {
                    b0 = 0;
                    if (status < 0xF0)
                        running = (unsigned char)status;
                }
                if (status == 0xFF) {
                    unsigned type;
                    unsigned long mlen = 0;
                    if (!r_u8(&tr, &type) || !r_vlq(&tr, &mlen))
                        die("bad meta event", in_path);
                    if (tr.pos + mlen > tr.size)
                        die("truncated meta event", in_path);
                    if (type == 0x51 && mlen == 3) {
                        Tempo tp;
                        tp.tick = track_tick[t];
                        tp.tempo_us = ((unsigned long)tr.buf[tr.pos] << 16) |
                                      ((unsigned long)tr.buf[tr.pos + 1] << 8) |
                                      (unsigned long)tr.buf[tr.pos + 2];
                        tp.cycle = 0;
                        if (tempos_cap == ntempos) {
                            size_t ncap = tempos_cap ? tempos_cap * 2 : 64;
                            Tempo *nt = (Tempo *)realloc(tempos, ncap * sizeof(Tempo));
                            if (!nt)
                                die("out of memory", NULL);
                            tempos = nt;
                            tempos_cap = ncap;
                        }
                        tempos[ntempos++] = tp;
                    }
                    tr.pos += mlen;
                } else if (status == 0xF0 || status == 0xF7) {
                    unsigned long slen;
                    unsigned long j;
                    Event ev;
                    if (!r_vlq(&tr, &slen))
                        die("bad sysex event", in_path);
                    if (tr.pos + slen > tr.size)
                        die("truncated sysex event", in_path);
                    ev.track = t;
                    ev.seq = seq++;
                    ev.tick = track_tick[t];
                    ev.cycle = 0;
                    ev.sysex = 1;
                    ev.nbytes = 0;
                    if (status == 0xF0) {
                        if (ev.nbytes < MAX_EVENT_BYTES)
                            ev.bytes[ev.nbytes++] = 0xF0;
                    }
                    if (slen > (unsigned long)(MAX_EVENT_BYTES - ev.nbytes))
                        die("sysex event too large", in_path);
                    for (j = 0; j < slen; j++)
                        ev.bytes[ev.nbytes++] = tr.buf[tr.pos + j];
                    if (status == 0xF0 && ev.nbytes > 0 && ev.bytes[ev.nbytes - 1] != 0xF7) {
                        if (ev.nbytes >= MAX_EVENT_BYTES)
                            die("sysex event too large", in_path);
                        ev.bytes[ev.nbytes++] = 0xF7;
                    }
                    tr.pos += slen;
                    if (events_cap == nevents) {
                        size_t ncap = events_cap ? events_cap * 2 : 1024;
                        Event *ne = (Event *)realloc(events, ncap * sizeof(Event));
                        if (!ne)
                            die("out of memory", NULL);
                        events = ne;
                        events_cap = ncap;
                    }
                    events[nevents++] = ev;
                } else if ((status & 0xF0) >= 0x80 && (status & 0xF0) <= 0xE0) {
                    int dbytes = ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) ? 1 : 2;
                    Event ev;
                    int k;
                    ev.track = t;
                    ev.seq = seq++;
                    ev.tick = track_tick[t];
                    ev.cycle = 0;
                    ev.sysex = 0;
                    ev.nbytes = 0;
                    ev.bytes[ev.nbytes++] = (unsigned char)status;
                    if (b0)
                        ev.bytes[ev.nbytes++] = (unsigned char)b0;
                    for (k = (b0 ? 1 : 0); k < dbytes; k++) {
                        unsigned db;
                        if (!r_u8(&tr, &db))
                            die("truncated channel event", in_path);
                        if (ev.nbytes < 8)
                            ev.bytes[ev.nbytes++] = (unsigned char)db;
                    }
                    if (events_cap == nevents) {
                        size_t ncap = events_cap ? events_cap * 2 : 1024;
                        Event *ne = (Event *)realloc(events, ncap * sizeof(Event));
                        if (!ne)
                            die("out of memory", NULL);
                        events = ne;
                        events_cap = ncap;
                    }
                    events[nevents++] = ev;
                } else {
                    die("unsupported status byte in track", in_path);
                }
            }
            r.pos += trk_len;
        }
    }

    if (ntempos == 0) {
        Tempo tp;
        tp.tick = 0;
        tp.tempo_us = 500000UL;
        tp.cycle = 0;
        tempos = (Tempo *)malloc(sizeof(Tempo));
        if (!tempos)
            die("out of memory", NULL);
        tempos[ntempos++] = tp;
    }
    qsort(tempos, ntempos, sizeof(Tempo), cmp_tempo);
    if (tempos[0].tick > 0) {
        Tempo *nt = (Tempo *)realloc(tempos, (ntempos + 1) * sizeof(Tempo));
        if (!nt)
            die("out of memory", NULL);
        tempos = nt;
        memmove(&tempos[1], &tempos[0], ntempos * sizeof(Tempo));
        tempos[0].tick = 0;
        tempos[0].tempo_us = 500000UL;
        tempos[0].cycle = 0;
        ntempos++;
    }
    if (!smpte) {
        unsigned long long base_tick = tempos[0].tick;
        unsigned long long base_cycle = 0;
        size_t k;
        for (k = 0; k < ntempos; k++) {
            if (k > 0) {
                base_cycle += (tempos[k].tick - base_tick) * 24ULL * tempos[k - 1].tempo_us /
                              (unsigned long long)ppq;
                base_tick = tempos[k].tick;
            }
            tempos[k].cycle = base_cycle;
        }
    }

    for (i = 0; i < nevents; i++) {
        unsigned long long tick = events[i].tick;
        size_t k;
        unsigned long long cyc = 0;
        if (smpte) {
            cyc = tick * 24000000ULL / ((unsigned long long)fps * tpf);
        } else {
            const Tempo *cur = &tempos[0];
            for (k = 0; k < ntempos; k++) {
                if (tempos[k].tick <= tick)
                    cur = &tempos[k];
                else
                    break;
            }
            cyc = cur->cycle +
                  (tick - cur->tick) * 24ULL * cur->tempo_us / (unsigned long long)ppq;
        }
        events[i].cycle = cyc;
    }
    qsort(events, nevents, sizeof(Event), cmp_event);

    /* Serialize like a real MIDI cable: bytes of one event are emitted
     * back-to-back (one byte_gap apart); an event may only start once the wire
     * is free. The previous event-local pacing interleaved the bytes of events
     * sharing a tick (A0 B0 A1 B1 ...), which desynchronized the firmware's
     * MIDI parser: patches/drum kits were lost and "everything is piano". */
    {
        unsigned long long wire = 0; /* next free serial byte time */
        for (i = 0; i < nevents; i++) {
            int b;
            unsigned long long t = events[i].cycle;
            if (t < wire)
                t = wire;
            for (b = 0; b < events[i].nbytes; b++) {
                push_byte(&lines, &nlines, &lines_cap, t, (int)i, b,
                          events[i].bytes[b]);
                t += byte_gap;
            }
            wire = t;
        }
    }
    qsort(lines, nlines, sizeof(OutLine), cmp_outline);

    if (out_path) {
        out = fopen(out_path, "wb");
        if (!out)
            die("cannot write output", out_path);
    } else {
        out = stdout;
    }

    fprintf(out, "# midisched v1 format=%u tracks=%u ppq=%d smpte=%d byte_gap=%llu\n",
            format, ntrks, ppq, smpte, byte_gap);
    {
        unsigned long long prev = 0;
        unsigned long long prev_cyc = 0;
        int prev_seq = -1;
        int first = 1;
        for (i = 0; i < nlines; i++) {
            unsigned long long cyc = lines[i].cycle + start;
            if (cyc < prev)
                cyc = prev;
            if (first || lines[i].seq != prev_seq || cyc != prev_cyc) {
                if (!first)
                    fputc('\n', out);
                if (verbose)
                    fprintf(out, "# evt[%d]\n", lines[i].seq);
                fprintf(out, "%llu %02x", cyc, lines[i].byte);
                first = 0;
            } else {
                fprintf(out, " %02x", lines[i].byte);
            }
            prev = cyc;
            prev_cyc = cyc;
            prev_seq = lines[i].seq;
        }
        if (!first)
            fputc('\n', out);
    }
    if (out != stdout && fclose(out) != 0)
        die("cannot write output", out_path);

    free(track_tick);
    free(events);
    free(tempos);
    free(lines);
    free(buf);
    return 0;
}
