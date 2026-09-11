#define _CRT_SECURE_NO_WARNINGS

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PCMDIFF_MAX_PRINT 64

typedef struct {
    const unsigned char *payload;
    size_t payload_size;
    unsigned format;
    unsigned channels;
    unsigned bits;
    unsigned rate;
    size_t samples;
} WavData;

typedef struct {
    long long index;
    long long ref;
    long long dut;
    long long delta;
} DiffSample;

typedef struct {
    unsigned long long frames_ref;
    unsigned long long frames_dut;
    unsigned long long samples_compared;
    unsigned long long diffs;
    long long max_abs_delta;
    unsigned long long sum_abs_delta;
    double sum_sq_ref;
    double sum_sq_dut;
    long long peak_ref;
    long long peak_dut;
    long long first_index;
    long long first_ref;
    long long first_dut;
    int length_mismatch;
} CompareStats;

typedef struct {
    unsigned long long first_cycle;
    unsigned rate;
    unsigned channels;
    int have_first_cycle;
} MetaInfo;

static void usage(FILE *out, const char *prog)
{
    fprintf(out,
            "usage: %s --ref <ref.wav> --dut <dut.wav> [--tolerance LSB]\n"
            "          [--stats] [--report out.md] [--max-print N]\n"
            "          [--meta-ref file] [--meta-dut file]\n"
            "\n"
            "exit: 0 equal within tolerance, 1 differences, 2 error\n",
            prog);
}

static unsigned long long read_u32le(const unsigned char *p)
{
    return (unsigned long long)p[0] | ((unsigned long long)p[1] << 8) |
           ((unsigned long long)p[2] << 16) | ((unsigned long long)p[3] << 24);
}

static unsigned read_u16le(const unsigned char *p)
{
    return (unsigned)p[0] | ((unsigned)p[1] << 8);
}

static unsigned char *read_file(const char *path, size_t *out_len)
{
    FILE *f;
    long long len;
    unsigned char *buf;
    size_t got;

    f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "pcmdiff: cannot open '%s': %s\n", path, strerror(errno));
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "pcmdiff: cannot seek '%s'\n", path);
        fclose(f);
        return NULL;
    }
    len = ftell(f);
    if (len < 0) {
        fprintf(stderr, "pcmdiff: cannot tell '%s'\n", path);
        fclose(f);
        return NULL;
    }
    if (fseek(f, 0, SEEK_SET) != 0) {
        fprintf(stderr, "pcmdiff: cannot rewind '%s'\n", path);
        fclose(f);
        return NULL;
    }
    buf = (unsigned char *)malloc((size_t)len + 1);
    if (!buf) {
        fprintf(stderr, "pcmdiff: out of memory\n");
        fclose(f);
        return NULL;
    }
    got = fread(buf, 1, (size_t)len, f);
    if (got != (size_t)len) {
        fprintf(stderr, "pcmdiff: short read on '%s'\n", path);
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *out_len = (size_t)len;
    return buf;
}

static int parse_wav(const unsigned char *buf, size_t len, const char *path,
                     WavData *wav)
{
    size_t pos = 12;
    int have_fmt = 0;
    int have_data = 0;

    memset(wav, 0, sizeof(*wav));
    if (len < 12 || memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) {
        fprintf(stderr, "pcmdiff: '%s' is not a RIFF/WAVE file\n", path);
        return 0;
    }
    while (pos + 8 <= len) {
        const unsigned char *id = buf + pos;
        unsigned long long size = read_u32le(buf + pos + 4);
        size_t body = pos + 8;
        if (size > (unsigned long long)(len - body)) {
            fprintf(stderr, "pcmdiff: '%s' has a truncated chunk at offset %llu\n",
                    path, (unsigned long long)pos);
            return 0;
        }
        if (memcmp(id, "fmt ", 4) == 0) {
            if (size < 16) {
                fprintf(stderr, "pcmdiff: '%s' has a short fmt chunk\n", path);
                return 0;
            }
            wav->format = read_u16le(buf + body + 0);
            wav->channels = read_u16le(buf + body + 2);
            wav->rate = (unsigned)read_u32le(buf + body + 4);
            wav->bits = read_u16le(buf + body + 14);
            have_fmt = 1;
        } else if (memcmp(id, "data", 4) == 0) {
            wav->payload = buf + body;
            wav->payload_size = (size_t)size;
            have_data = 1;
        }
        pos = body + (size_t)size + ((size & 1) ? 1 : 0);
    }
    if (!have_fmt || !have_data) {
        fprintf(stderr, "pcmdiff: '%s' missing %s chunk\n", path,
                have_fmt ? "data" : "fmt ");
        return 0;
    }
    if (wav->format != 1) {
        fprintf(stderr,
                "pcmdiff: '%s' unsupported WAV format tag %u (expected 1 = PCM)\n",
                path, wav->format);
        return 0;
    }
    if (wav->bits != 8 && wav->bits != 16 && wav->bits != 24 && wav->bits != 32) {
        fprintf(stderr, "pcmdiff: '%s' unsupported PCM bit depth %u\n", path,
                wav->bits);
        return 0;
    }
    if (wav->channels == 0) {
        fprintf(stderr, "pcmdiff: '%s' has zero channels\n", path);
        return 0;
    }
    {
        unsigned bytes = wav->bits / 8;
        size_t frame = (size_t)bytes * wav->channels;
        if (wav->payload_size % frame != 0) {
            fprintf(stderr,
                    "pcmdiff: '%s' payload %llu bytes is not a whole number of frames\n",
                    path, (unsigned long long)wav->payload_size);
            return 0;
        }
        wav->samples = wav->payload_size / bytes;
    }
    return 1;
}

static long long sample_at(const WavData *wav, size_t index)
{
    const unsigned char *p = wav->payload + index * (wav->bits / 8);
    switch (wav->bits) {
    case 8:
        return (long long)((int)p[0] - 128);
    case 16:
        return (long long)(short)((unsigned)p[0] | ((unsigned)p[1] << 8));
    case 24: {
        long v = (long)p[0] | ((long)p[1] << 8) | ((long)p[2] << 16);
        if (v & 0x800000L)
            v -= 0x1000000L;
        return (long long)v;
    }
    case 32:
        return (long long)(int)((unsigned long long)p[0] |
                                ((unsigned long long)p[1] << 8) |
                                ((unsigned long long)p[2] << 16) |
                                ((unsigned long long)p[3] << 24));
    default:
        return 0;
    }
}

static int read_meta(const char *path, MetaInfo *meta)
{
    FILE *f;
    char line[512];
    char key[128];
    char val[256];

    memset(meta, 0, sizeof(*meta));
    f = fopen(path, "rb");
    if (!f)
        return 0;
    while (fgets(line, sizeof(line), f)) {
        key[0] = '\0';
        val[0] = '\0';
        if (sscanf(line, " %127[A-Za-z0-9_.] = %255s", key, val) != 2)
            continue;
        if (strcmp(key, "rate") == 0)
            meta->rate = (unsigned)strtoul(val, NULL, 10);
        else if (strcmp(key, "channels") == 0)
            meta->channels = (unsigned)strtoul(val, NULL, 10);
        else if (strcmp(key, "first_cycle") == 0) {
            meta->first_cycle = strtoull(val, NULL, 10);
            meta->have_first_cycle = 1;
        } else if (strcmp(key, "start_cycles") == 0 && !meta->have_first_cycle) {
            meta->first_cycle = strtoull(val, NULL, 10);
            meta->have_first_cycle = 1;
        }
    }
    fclose(f);
    return 1;
}

static int parse_ll(const char *s, long long *out)
{
    char *end;
    long long v;
    errno = 0;
    v = strtoll(s, &end, 10);
    if (end == s || *end || errno == ERANGE)
        return 0;
    *out = v;
    return 1;
}

static void report_markdown(FILE *f, const char *ref_path, const char *dut_path,
                            const WavData *ref, long long tolerance,
                            const CompareStats *st,
                            const DiffSample *first, size_t first_count,
                            int equal)
{
    unsigned long long frames;
    double length_s;

    fprintf(f, "# pcmdiff report\n\n");
    fprintf(f, "- ref: `%s`\n", ref_path);
    fprintf(f, "- dut: `%s`\n", dut_path);
    fprintf(f, "- format: PCM, channels=%u, bits=%u, rate=%u\n", ref->channels,
            ref->bits, ref->rate);
    fprintf(f, "- frames: ref=%llu, dut=%llu\n", st->frames_ref, st->frames_dut);
    frames = st->frames_ref < st->frames_dut ? st->frames_ref : st->frames_dut;
    length_s = ref->rate ? (double)frames / (double)ref->rate : 0.0;
    fprintf(f, "- duration (shorter side): %.6f s\n", length_s);
    fprintf(f, "- tolerance: %lld LSB\n", tolerance);
    fprintf(f, "- samples compared: %llu\n", st->samples_compared);
    fprintf(f, "- differing samples: %llu (%.6f%%)\n", st->diffs,
            st->samples_compared
                ? 100.0 * (double)st->diffs / (double)st->samples_compared
                : 0.0);
    if (st->length_mismatch)
        fprintf(f, "- WARNING: payload lengths differ (tail counted as differences)\n");
    if (st->first_index >= 0) {
        fprintf(f, "- first difference: sample %lld (ref=%lld dut=%lld delta=%lld)\n",
                st->first_index, st->first_ref, st->first_dut,
                st->first_dut - st->first_ref);
    } else {
        fprintf(f, "- first difference: none\n");
    }
    fprintf(f, "- max |delta|: %lld\n", st->max_abs_delta);
    fprintf(f, "- sum |delta|: %llu\n", st->sum_abs_delta);
    fprintf(f, "- RMS: ref=%.3f dut=%.3f\n",
            st->samples_compared
                ? sqrt(st->sum_sq_ref / (double)st->samples_compared)
                : 0.0,
            st->samples_compared
                ? sqrt(st->sum_sq_dut / (double)st->samples_compared)
                : 0.0);
    fprintf(f, "- peak |sample|: ref=%lld dut=%lld\n", st->peak_ref, st->peak_dut);
    fprintf(f, "- result: %s\n", equal ? "equal within tolerance" : "DIFFER");
    if (first_count > 0) {
        size_t i;
        fprintf(f, "\n## First differing samples\n\n");
        fprintf(f, "| sample | ref | dut | delta |\n");
        fprintf(f, "|---:|---:|---:|---:|\n");
        for (i = 0; i < first_count; i++)
            fprintf(f, "| %lld | %lld | %lld | %lld |\n", first[i].index,
                    first[i].ref, first[i].dut,
                    first[i].dut - first[i].ref);
    }
}

int main(int argc, char **argv)
{
    const char *ref_path = NULL;
    const char *dut_path = NULL;
    const char *report_path = NULL;
    const char *meta_ref_path = NULL;
    const char *meta_dut_path = NULL;
    long long tolerance = 0;
    long long max_print = 8;
    int stats_only = 0;
    int i;
    unsigned char *ref_buf = NULL;
    unsigned char *dut_buf = NULL;
    size_t ref_len = 0, dut_len = 0;
    WavData ref, dut;
    CompareStats st;
    DiffSample first[PCMDIFF_MAX_PRINT];
    size_t first_count = 0;
    long long tolerance_arg = 0;
    MetaInfo meta_ref, meta_dut;
    char auto_meta[4096];
    int equal;
    int status = 2;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--ref") == 0 && i + 1 < argc) {
            ref_path = argv[++i];
        } else if (strcmp(argv[i], "--dut") == 0 && i + 1 < argc) {
            dut_path = argv[++i];
        } else if (strcmp(argv[i], "--tolerance") == 0 && i + 1 < argc) {
            if (!parse_ll(argv[++i], &tolerance_arg) || tolerance_arg < 0) {
                fprintf(stderr, "pcmdiff: invalid --tolerance value\n");
                return 2;
            }
            tolerance = tolerance_arg;
        } else if (strcmp(argv[i], "--max-print") == 0 && i + 1 < argc) {
            if (!parse_ll(argv[++i], &max_print) || max_print < 0 ||
                max_print > PCMDIFF_MAX_PRINT) {
                fprintf(stderr, "pcmdiff: invalid --max-print value (0..%d)\n",
                        PCMDIFF_MAX_PRINT);
                return 2;
            }
        } else if (strcmp(argv[i], "--report") == 0 && i + 1 < argc) {
            report_path = argv[++i];
        } else if (strcmp(argv[i], "--meta-ref") == 0 && i + 1 < argc) {
            meta_ref_path = argv[++i];
        } else if (strcmp(argv[i], "--meta-dut") == 0 && i + 1 < argc) {
            meta_dut_path = argv[++i];
        } else if (strcmp(argv[i], "--stats") == 0) {
            stats_only = 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else {
            usage(stderr, argv[0]);
            return 2;
        }
    }
    if (!ref_path || !dut_path) {
        usage(stderr, argv[0]);
        return 2;
    }

    ref_buf = read_file(ref_path, &ref_len);
    if (!ref_buf)
        goto cleanup;
    dut_buf = read_file(dut_path, &dut_len);
    if (!dut_buf)
        goto cleanup;
    if (!parse_wav(ref_buf, ref_len, ref_path, &ref))
        goto cleanup;
    if (!parse_wav(dut_buf, dut_len, dut_path, &dut))
        goto cleanup;

    if (ref.channels != dut.channels || ref.bits != dut.bits ||
        ref.rate != dut.rate) {
        fprintf(stderr,
                "pcmdiff: format mismatch: ref ch=%u bits=%u rate=%u vs dut ch=%u bits=%u rate=%u\n",
                ref.channels, ref.bits, ref.rate, dut.channels, dut.bits,
                dut.rate);
        goto cleanup;
    }

    if (meta_ref_path) {
        read_meta(meta_ref_path, &meta_ref);
    } else {
        snprintf(auto_meta, sizeof(auto_meta), "%s.meta", ref_path);
        read_meta(auto_meta, &meta_ref);
    }
    if (meta_dut_path) {
        read_meta(meta_dut_path, &meta_dut);
    } else {
        snprintf(auto_meta, sizeof(auto_meta), "%s.meta", dut_path);
        read_meta(auto_meta, &meta_dut);
    }
    if (!meta_ref.have_first_cycle && meta_dut.have_first_cycle)
        meta_ref = meta_dut;
    if (!meta_ref.rate)
        meta_ref.rate = ref.rate;
    if (!meta_ref.channels)
        meta_ref.channels = ref.channels;

    memset(&st, 0, sizeof(st));
    st.first_index = -1;
    {
        size_t n_ref = ref.samples;
        size_t n_dut = dut.samples;
        size_t n = n_ref < n_dut ? n_ref : n_dut;
        size_t k;
        st.frames_ref = n_ref / ref.channels;
        st.frames_dut = n_dut / dut.channels;
        st.length_mismatch = (n_ref != n_dut);
        for (k = 0; k < n; k++) {
            long long a = sample_at(&ref, k);
            long long b = sample_at(&dut, k);
            long long d = b - a;
            long long ad = d < 0 ? -d : d;
            double da = (double)a;
            double db = (double)b;
            st.samples_compared++;
            st.sum_sq_ref += da * da;
            st.sum_sq_dut += db * db;
            if (llabs(a) > st.peak_ref)
                st.peak_ref = llabs(a);
            if (llabs(b) > st.peak_dut)
                st.peak_dut = llabs(b);
            if (ad > 0) {
                st.sum_abs_delta += (unsigned long long)ad;
                if (ad > st.max_abs_delta)
                    st.max_abs_delta = ad;
                if (ad > tolerance) {
                    st.diffs++;
                    if (st.first_index < 0) {
                        st.first_index = (long long)k;
                        st.first_ref = a;
                        st.first_dut = b;
                    }
                    if (first_count < (size_t)max_print) {
                        first[first_count].index = (long long)k;
                        first[first_count].ref = a;
                        first[first_count].dut = b;
                        first[first_count].delta = d;
                        first_count++;
                    }
                }
            }
        }
        if (n_dut != n_ref) {
            unsigned long long tail = (unsigned long long)(n_ref > n_dut ? n_ref - n_dut
                                                                        : n_dut - n_ref);
            st.diffs += tail;
        }
    }

    equal = !st.length_mismatch && st.diffs == 0;

    printf("pcmdiff: ref='%s' dut='%s'\n", ref_path, dut_path);
    printf("pcmdiff: format ch=%u bits=%u rate=%u frames ref=%llu dut=%llu\n",
           ref.channels, ref.bits, ref.rate, st.frames_ref, st.frames_dut);
    printf("pcmdiff: samples compared=%llu diffs=%llu max|d|=%lld tolerance=%lld\n",
           st.samples_compared, st.diffs, st.max_abs_delta, tolerance);
    if (st.length_mismatch) {
        printf("pcmdiff: length mismatch (tail counted as differences)\n");
        printf("pcmdiff: result = DIFFER\n");
    } else if (st.diffs == 0) {
        printf("pcmdiff: result = equal within tolerance\n");
    } else {
        printf("pcmdiff: first diff sample=%lld ref=%lld dut=%lld\n",
               st.first_index, st.first_ref, st.first_dut);
        if (meta_ref.have_first_cycle && meta_ref.rate > 0 && ref.channels > 0) {
            double cycles = (double)meta_ref.first_cycle +
                            ((double)st.first_index / (double)ref.channels /
                             (double)meta_ref.rate) *
                                24000000.0;
            printf("pcmdiff: first diff est cycle=%.0f\n", cycles);
        }
        printf("pcmdiff: result = DIFFER (%llu samples beyond tolerance)\n", st.diffs);
    }

    if (report_path) {
        FILE *rf = fopen(report_path, "wb");
        if (!rf) {
            fprintf(stderr, "pcmdiff: cannot write '%s': %s\n", report_path,
                    strerror(errno));
            goto cleanup;
        }
        report_markdown(rf, ref_path, dut_path, &ref, tolerance, &st, first,
                        first_count, equal);
        if (fclose(rf) != 0) {
            fprintf(stderr, "pcmdiff: cannot write '%s': %s\n", report_path,
                    strerror(errno));
            goto cleanup;
        }
    }

    if (stats_only)
        status = 0;
    else
        status = equal ? 0 : 1;

cleanup:
    free(ref_buf);
    free(dut_buf);
    return status;
}
