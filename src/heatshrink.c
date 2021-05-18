#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>

#include "heatshrink_encoder.h"
#include "heatshrink_decoder.h"

#define DEF_OUTPUT_CHUNK_SIZE 64

#define DEF_WINDOW_SZ2 11
#define DEF_LOOKAHEAD_SZ2 4
#define DEF_DECODER_INPUT_BUFFER_SIZE 256
#define DEF_BUFFER_SIZE (64 * 1024)

#if 0
#define LOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define LOG(...) /* NO-OP */
#endif

#if _WIN32
#include <errno.h>
#define HEATSHRINK_ERR(retval, ...) do { \
fprintf(stderr, __VA_ARGS__); \
fprintf(stderr, "Undefined error: %d\n", errno); \
exit(retval); \
} while(0)
#else
#include <err.h>
#define HEATSHRINK_ERR(...) err(__VA_ARGS__)
#endif

/*
 * We have to open binary files with the O_BINARY flag on Windows. Most other
 * platforms don't differentiate between binary and non-binary files.
 */
#ifndef O_BINARY
#define O_BINARY 0
#endif

static const int version_major = HEATSHRINK_VERSION_MAJOR;
static const int version_minor = HEATSHRINK_VERSION_MINOR;
static const int version_patch = HEATSHRINK_VERSION_PATCH;
static const char author[] = HEATSHRINK_AUTHOR;
static const char url[] = HEATSHRINK_URL;

static void usage(void) {
    fprintf(stderr, "heatshrink version %u.%u.%u by %s\n",
        version_major, version_minor, version_patch, author);
    fprintf(stderr, "Home page: %s\n\n", url);
    fprintf(stderr,
        "Usage:\n"
        "  heatshrink [-h] [-e|-d] [-v] [-w SIZE] [-l BITS] [IN_FILE] [OUT_FILE]\n"
        "\n"
        "heatshrink compresses or decompresses byte streams using LZSS, and is\n"
        "designed especially for embedded, low-memory, and/or hard real-time\n"
        "systems.\n"
        "\n"
        " -h        print help\n"
        " -e        encode (compress, default)\n"
        " -d        decode (decompress)\n"
        " -v        verbose (print input & output sizes, compression ratio, etc.)\n"
        "\n"
        " -w SIZE   Base-2 log of LZSS sliding window size\n"
        "\n"
        "    A larger value allows searches a larger history of the data for repeated\n"
        "    patterns, potentially compressing more effectively, but will use\n"
        "    more memory and processing time.\n"
        "    Recommended default: -w 8 (embedded systems), -w 10 (elsewhere)\n"
        "  \n"
        " -l BITS   Number of bits used for back-reference lengths\n"
        "\n"
        "    A larger value allows longer substitutions, but since all\n"
        "    back-references must use -w + -l bits, larger -w or -l can be\n"
        "    counterproductive if most patterns are small and/or local.\n"
        "    Recommended default: -l 4\n"
        "\n"
        " If IN_FILE or OUT_FILE are unspecified, they will default to\n"
        " \"-\" for standard input and standard output, respectively.\n");
    exit(1);
}

typedef enum { IO_READ, IO_WRITE, } IO_mode;
typedef enum { OP_ENC, OP_DEC, } Operation;

typedef struct {
    int fd;                     /* file descriptor */
    IO_mode mode;
    size_t fill;                /* fill index */
    size_t read;                /* read index */
    size_t size;
    size_t total;
    uint8_t buf[];
} io_handle;

typedef struct {
    uint8_t window_sz2;
    uint8_t lookahead_sz2;
    size_t decoder_input_buffer_size;
    size_t buffer_size;
    uint8_t verbose;
    Operation cmd;
    char *in_fname;
    char *out_fname;
    io_handle *in;
    io_handle *out;
} config;

static void die(char *msg) {
    fprintf(stderr, "%s\n", msg);
    exit(EXIT_FAILURE);
}

static void report(config *cfg);

/* Open an IO handle. Returns NULL on error. */
static io_handle *handle_open(char *fname, IO_mode m, size_t buf_sz) {
    io_handle *io = NULL;
    io = malloc(sizeof(*io) + buf_sz);
    if (io == NULL) { return NULL; }
    memset(io, 0, sizeof(*io) + buf_sz);
    io->fd = -1;
    io->size = buf_sz;
    io->mode = m;

    if (m == IO_READ) {
        if (0 == strcmp("-", fname)) {
            io->fd = STDIN_FILENO;
        } else {
            io->fd = open(fname, O_RDONLY | O_BINARY);
        }
    } else if (m == IO_WRITE) {
        if (0 == strcmp("-", fname)) {
            io->fd = STDOUT_FILENO;
        } else {
            io->fd = open(fname, O_WRONLY | O_BINARY | O_CREAT | O_TRUNC /*| O_EXCL*/, 0644);
        }
    }

    if (io->fd == -1) {         /* failed to open */
        free(io);
        HEATSHRINK_ERR(1, "open");
        return NULL;
    }

    return io;
}

/* Read SIZE bytes from an IO handle and return a pointer to the content.
 * BUF contains at least size_t bytes. Returns 0 on EOF, -1 on error. */
static ssize_t handle_read(io_handle *io, size_t size, uint8_t **buf) {
    LOG("@ read %zd\n", size);
    if (buf == NULL) { return -1; }
    if (size > io->size) {
        fprintf(stderr, "size %zd, io->size %zd\n", size, io->size);
        return -1;
    }
    if (io->mode != IO_READ) { return -1; }

    size_t rem = io->fill - io->read;
    if (rem >= size) {
        *buf = &io->buf[io->read];
        return size;
    } else {                    /* read and replenish */
        if (io->fd == -1) {     /* already closed, return what we've got */
            *buf = &io->buf[io->read];
            return rem;
        }

        memmove(io->buf, &io->buf[io->read], rem);
        io->fill -= io->read;
        io->read = 0;
        ssize_t read_sz = read(io->fd, &io->buf[io->fill], io->size - io->fill);
        if (read_sz < 0) { HEATSHRINK_ERR(1, "read"); }
        io->total += read_sz;
        if (read_sz == 0) {     /* EOF */
            if (close(io->fd) < 0) { HEATSHRINK_ERR(1, "close"); }
            io->fd = -1;
        }
        io->fill += read_sz;
        *buf = io->buf;
        return io->fill > size ? size : io->fill;
    }
}

/* Drop the oldest SIZE bytes from the buffer. Returns <0 on error. */
static int handle_drop(io_handle *io, size_t size) {
    LOG("@ drop %zd\n", size);
    if (io->read + size <= io->fill) {
        io->read += size;
    } else {
        return -1;
    }
    if (io->read == io->fill) {
        io->read = 0;
        io->fill = 0;
    }
    return 0;
}

/* Sink SIZE bytes from INPUT into the io handle. Returns the number of
 * bytes written, or -1 on error. */
static ssize_t handle_sink(io_handle *io, size_t size, uint8_t *input) {
    LOG("@ sink %zd\n", size);
    if (size > io->size) { return -1; }
    if (io->mode != IO_WRITE) { return -1; }

    if (io->fill + size > io->size) {
        ssize_t written = write(io->fd, io->buf, io->fill);
        LOG("@ flushing %zd, wrote %zd\n", io->fill, written);
        io->total += written;
        if (written == -1) { HEATSHRINK_ERR(1, "write"); }
        memmove(io->buf, &io->buf[written], io->fill - written);
        io->fill -= written;
    }
    memcpy(&io->buf[io->fill], input, size);
    io->fill += size;
    return size;
}

static void handle_close(io_handle *io) {
    if (io->fd != -1) {
        if (io->mode == IO_WRITE) {
            ssize_t written = write(io->fd, io->buf, io->fill);
            io->total += written;
            LOG("@ close: flushing %zd, wrote %zd\n", io->fill, written);
            if (written == -1) { HEATSHRINK_ERR(1, "write"); }
        }
        close(io->fd);
        io->fd = -1;
    }
}

static void close_and_report(config *cfg) {
    handle_close(cfg->in);
    handle_close(cfg->out);
    if (cfg->verbose) { report(cfg); }
    free(cfg->in);
    free(cfg->out);
}

#include <stdbool.h>
/* File transfer state structure. */
struct send_compressed_file_state {
	heatshrink_encoder *hse;
	int in_fd;
	bool is_done;
	bool poll_empty;

#ifdef __HS_CONTEXT_NO_INPUT_BUFF__
	uint8_t * in_buf;  // Data Buffer - data - needs to be periodically refreshed from the file - its then transfer into the HSE window.
	size_t in_buf_sz;  // Data Buffer Size - data_sz
#endif //#ifdef __HS_CONTEXT_NO_INPUT_BUFF__
	int out_fd;  // TODO Used for testing only - will be removed in streamed version.
};

#define ALLOC malloc
#define FREE free

struct send_compressed_file_state * AEBusHost_allocHsContext(const char * filePath, const uint8_t window_sz2, const uint8_t lookahead_sz2);
void AEBusHost_freeHsContext(struct send_compressed_file_state * hs_context);
int hs_encode_initial_step(struct send_compressed_file_state * hs_context);
int encode_step(struct send_compressed_file_state * hs_context, uint8_t * out_buf, uint16_t out_buf_len, size_t *out_buf_sz);

struct send_compressed_file_state * AEBusHost_allocHsContext(const char * filePath, const uint8_t window_sz2, const uint8_t lookahead_sz2)
{
	struct send_compressed_file_state * hs_context = (struct send_compressed_file_state *)ALLOC(sizeof(struct send_compressed_file_state));
	if(hs_context == NULL)
	{
		return (NULL);
	}

	memset(hs_context, 0, sizeof(*hs_context));

	hs_context->in_fd = open(filePath, ( O_RDONLY | O_BINARY));
	if(hs_context->in_fd < 0)
	{
		FREE(hs_context);

		return (NULL);
	}

    hs_context->hse = heatshrink_encoder_alloc(window_sz2, lookahead_sz2);
    if (hs_context->hse == NULL)
    {
    	FREE(hs_context);

    	return (NULL);
    }

    hs_context->is_done = false;
    hs_context->poll_empty = true;

#ifdef __HS_CONTEXT_NO_INPUT_BUFF__
    size_t window_sz = 1 << window_sz2;
	hs_context->in_buf = ALLOC(window_sz);
	if (hs_context->in_buf == NULL)
	{
		heatshrink_encoder_free(hs_context->hse);
    	FREE(hs_context);

    	return (NULL);
	}
	hs_context->in_buf_sz = window_sz;
#endif //#ifdef __HS_CONTEXT_NO_INPUT_BUFF__
	return (hs_context);
}


void AEBusHost_freeHsContext(struct send_compressed_file_state * hs_context)
{
	assert(hs_context != NULL);

	if(hs_context->in_fd >= 0)
	{
		/* Close the file. */
		close(hs_context->in_fd);
	}

#ifdef __HS_CONTEXT_NO_INPUT_BUFF__
	FREE(hs_context->in_buf);
#endif //#ifdef __HS_CONTEXT_NO_INPUT_BUFF__
	heatshrink_encoder_free(hs_context->hse);
	FREE(hs_context);
}
#if 0
/* This first step  in the encoder if meant to read the code for the iterative
 * encoding. Prime all input buffers and the encoder window buffer. This is
 * a time saving function as it avoids buffering file input to an input buffer
 * and then copying it to the encoder state machine. This should only be used
 * on the first iteration of the state machine.
 *
 * Steps:
 * 1. Prime the encoder by filling its internal buffer with content from the
 *    file.
 * 2. Check for EOF while filling the initial buffer - if found we are done.
 * 3. Fill the file read buffer
 * Then fill the input buffer with content form the file.
 */
int hs_encode_initial_step(struct send_compressed_file_state * hs_context)
{
	size_t read_sz;
	do
	{
		read_sz = 0;
		if (HSER_SINK_OK != heatshrink_encoder_sink_file_read(hs_context->hse, hs_context->in_fd, &read_sz))
		{
			// ERROR
			return (-1);
		}
		/* Check for EOF - could be the window size is larger than the
		 * file. Since the internal read is using */
		if(read_sz == 0)
		{
			// EOF - Signal HSE that we are already done.
			//       Close the file, and mark the in_fd as closed.
			close(hs_context->in_fd);
			hs_context->in_fd = -1;
			break;
		}
	}while (hs_context->hse->state != 0); //HSES_NOT_FULL

	/* Success */
	return (0);
}
#endif //#if 0

/* This function replaces the encode() and encode_sink_read() - it will be called
 * on each poll of the file fragment from the AEBusHost file server handler.
 *
 * The function will read new input into the encoder sliding window buffer. Then
 * call the encoder state machine. Once the output buffer size worth of data is
 * encoded (compressed) then encoder state machine saves the state and the output
 * message data is returned.
 *
 * If the encoder has exhausted the input file and completed all encoding, a value
 * of 1 is returned. If not a value of 0 is returned. Errors of any kind are
 * returned as negative values.
 */
int encode_step(struct send_compressed_file_state * hs_context, uint8_t * out_buf, uint16_t out_buf_len, size_t *out_buf_sz)
{
	if (hs_context == NULL)
	{
		return (-1);
	}
	if (hs_context->hse == NULL)
	{
		return (-1);
	}
	heatshrink_encoder * hse = hs_context->hse;

	/* Read data from the file to replenish the input. This must be done on every
	 * iteration or there is a risk of underflow for the encoder HS state machine
	 * which result in it finishing. */
	if ((hs_context->in_fd != -1) && (hs_context->poll_empty))
	{
		size_t readFile_sz = 0;
		HSE_sink_res sres = heatshrink_encoder_sink_file_read(hse, hs_context->in_fd, &readFile_sz);
		/* Check for EOF */
		if(readFile_sz == 0)
		{
			// EOF - Signal HSE that we are already done.
			//       Close the file, and mark the in_fd as closed.
			close(hs_context->in_fd);
			hs_context->in_fd = -1;

			printf(" - Encode CLOSE file\n", sres);
		}
#if 0
		if(sres != HSER_SINK_OK)
#endif
		{
			printf(" - Encode Read file result = %ld, read %d bytes\n", sres, readFile_sz);
		}
		assert(sres >= HSER_SINK_OK);
		hs_context->poll_empty = false;
	}

	HSE_poll_res pres;
	size_t poll_sz = 0;
	size_t out_sz = 0;
	do
	{
		pres = heatshrink_encoder_poll(hse, &out_buf[out_sz], (out_buf_len - out_sz), &poll_sz);
		out_sz += poll_sz;
		if (pres == HSER_POLL_EMPTY)
		{
			hs_context->poll_empty = true;
		}
		else if (pres < HSER_POLL_EMPTY)
		{
			printf("ERROR Poll encoder result = %d\n", pres);
			assert(pres >= 0);
		}

		/* TODO: Check for case where we need to replenish the input buffer.
		 *       This would happen if the input buffer/window size is close to
		 *       the same size as the output buffer. Probably best to add a
		 *       check and error handler for this situation. Tuning the buffers
		 *       is probably good enough for now - also better to have this as
		 *       an assert in the initialization code rather than on each pass
		 *       of the encoder which is wasteful of CPU.
		 */

		/* Check if we are finished because the last fragment of the output
		 * maybe less than the chunk size. */
		if (hs_context->in_fd == -1)
		{
			/* Set the finish flag and poll the state machine for completion. */
			HSE_finish_res fres = heatshrink_encoder_finish(hse);
			if (fres == HSER_FINISH_DONE)
			{
				hs_context->is_done = true;
				break;
			}
			/* Check for errors. */
			if (fres < 0)
			{
				return (-3);
			}
		}

		/* Output only in 64 byte checks as though we were streaming */
		if (out_sz >= out_buf_len)
		{
			break;
		}
	} while (pres == HSER_POLL_MORE);

	*out_buf_sz = out_sz;

	return 0;
}

/* The read sink iterates over a buffer of input data. The data buffer is processed by repeatedly shifting the window forward */
static int encoder_sink_read(config *cfg, heatshrink_encoder *hse,
        uint8_t *data, size_t data_sz) {
    size_t out_sz = cfg->buffer_size;
    uint8_t out_buf[out_sz];
    memset(out_buf, 0, out_sz);
    size_t sink_sz = 0;
    size_t poll_sz = 0;
    HSE_sink_res sres;
    HSE_poll_res pres;
    HSE_finish_res fres;
    io_handle *out = cfg->out;

    size_t sunk = 0;
    do {
        if (data_sz > 0) {
            sres = heatshrink_encoder_sink(hse, &data[sunk], data_sz - sunk, &sink_sz);
            if (sres < 0) { die("sink"); }
            sunk += sink_sz;
        }
        
        do {
        	pres = heatshrink_encoder_poll(hse, out_buf, out_sz, &poll_sz);
        	if (pres < 0) { die("poll"); }
        	if (handle_sink(out, poll_sz, out_buf) < 0) die("handle_sink");
        } while (pres == HSER_POLL_MORE);
        
        if (poll_sz == 0 && data_sz == 0) {
            fres = heatshrink_encoder_finish(hse);
            if (fres < 0) { die("finish"); }
            if (fres == HSER_FINISH_DONE) { return 1; }
        }
    } while (sunk < data_sz);
    return 0;
}

static int encode(config *cfg) {

#if 1
	struct send_compressed_file_state * hs_context = AEBusHost_allocHsContext(cfg->in_fname, cfg->window_sz2, cfg->lookahead_sz2);

	/* Copy output file descriptor. */
	hs_context->out_fd = cfg->out->fd;
	/* <Optional> Fill the hs encoder with up to the window size bytes but don't start
	 * polling. This just keeps the amount of time spent doing file access down after
	 * the file transfer has started. */
	//hs_encode_initial_step(hs_context);

	int idx = 0;
	size_t out_sz = 0;
	uint8_t out_buf[DEF_OUTPUT_CHUNK_SIZE]; /* This is the output message size. */
	do
	{
		if (encode_step(hs_context, &out_buf[out_sz], DEF_OUTPUT_CHUNK_SIZE, &out_sz) < 0)
		{
			/* ERROR exit.*/
			printf("Encode step failed on iteration %d!!! Exiting...\n", idx);
			break;
		}
		printf("Encode step returned %ld bytes\n", out_sz);
		if((out_sz == DEF_OUTPUT_CHUNK_SIZE ) || (hs_context->is_done))
		{
			printf("\t Writing out %ld bytes\n", out_sz);
			/* The output buffer to the output file. */
			if (write(hs_context->out_fd, out_buf, out_sz) < 0)
			{
				return (-2);
			}
			out_sz = 0;
		}
		idx++;
	}while(hs_context->is_done == false);

	AEBusHost_freeHsContext(hs_context);
    return 0;
#else
    uint8_t window_sz2 = cfg->window_sz2;
    size_t window_sz = 1 << window_sz2; 
    heatshrink_encoder *hse = heatshrink_encoder_alloc(window_sz2, cfg->lookahead_sz2);
    if (hse == NULL) { die("failed to init encoder: bad settings"); }
    ssize_t read_sz = 0;
    io_handle *in = cfg->in;

    /* Process input until end of stream */
    while (1) {
        uint8_t *input = NULL;
        read_sz = handle_read(in, window_sz, &input);
        if (input == NULL) {
            fprintf(stderr, "handle read failure\n");
            die("read");
        }
        if (read_sz < 0) { die("read"); }

        /* Pass read to encoder and check if input is fully processed. */
        if (encoder_sink_read(cfg, hse, input, read_sz)) break;

        if (handle_drop(in, read_sz) < 0) { die("drop"); }
    };

    if (read_sz == -1) { HEATSHRINK_ERR(1, "read"); }

    heatshrink_encoder_free(hse);
    close_and_report(cfg);
    return 0;
#endif
}

static int decoder_sink_read(config *cfg, heatshrink_decoder *hsd,
        uint8_t *data, size_t data_sz) {
    io_handle *out = cfg->out;
    size_t sink_sz = 0;
    size_t poll_sz = 0;
    size_t out_sz = cfg->buffer_size;
    uint8_t out_buf[out_sz];
    memset(out_buf, 0, out_sz);

    HSD_sink_res sres;
    HSD_poll_res pres;
    HSD_finish_res fres;

    size_t sunk = 0;
    do {
        if (data_sz > 0) {
            sres = heatshrink_decoder_sink(hsd, &data[sunk], data_sz - sunk, &sink_sz);
            if (sres < 0) { die("sink"); }
            sunk += sink_sz;
        }

        do {
            pres = heatshrink_decoder_poll(hsd, out_buf, out_sz, &poll_sz);
            if (pres < 0) { die("poll"); }
            if (handle_sink(out, poll_sz, out_buf) < 0) die("handle_sink");
        } while (pres == HSDR_POLL_MORE);
        
        if (data_sz == 0 && poll_sz == 0) {
            fres = heatshrink_decoder_finish(hsd);
            if (fres < 0) { die("finish"); }
            if (fres == HSDR_FINISH_DONE) { return 1; }
        }
    } while (sunk < data_sz);

    return 0;
}

static int decode(config *cfg) {
    uint8_t window_sz2 = cfg->window_sz2;
    size_t window_sz = 1 << window_sz2;
    size_t ibs = cfg->decoder_input_buffer_size;
    heatshrink_decoder *hsd = heatshrink_decoder_alloc(ibs,
        window_sz2, cfg->lookahead_sz2);
    if (hsd == NULL) { die("failed to init decoder"); }

    ssize_t read_sz = 0;

    io_handle *in = cfg->in;

    HSD_finish_res fres;

    /* Process input until end of stream */
    while (1) {
        uint8_t *input = NULL;
        read_sz = handle_read(in, window_sz, &input);
        if (input == NULL) {
            fprintf(stderr, "handle read failure\n");
            die("read");
        }
        if (read_sz == 0) {
            fres = heatshrink_decoder_finish(hsd);
            if (fres < 0) { die("finish"); }
            if (fres == HSDR_FINISH_DONE) break;
        } else if (read_sz < 0) {
            die("read");
        } else {
            if (decoder_sink_read(cfg, hsd, input, read_sz)) { break; }
            if (handle_drop(in, read_sz) < 0) { die("drop"); }
        }
    }
    if (read_sz == -1) { HEATSHRINK_ERR(1, "read"); }
        
    heatshrink_decoder_free(hsd);
    close_and_report(cfg);
    return 0;
}

static void report(config *cfg) {
    size_t inb = cfg->in->total;
    size_t outb = cfg->out->total;
    fprintf(cfg->out->fd == STDOUT_FILENO ? stderr : stdout,
        "%s %0.2f %%\t %zd -> %zd (-w %u -l %u)\n",
        cfg->in_fname, 100.0 - (100.0 * outb) / inb, inb, outb,
        cfg->window_sz2, cfg->lookahead_sz2);
}

static void proc_args(config *cfg, int argc, char **argv) {
    cfg->window_sz2 = DEF_WINDOW_SZ2;
    cfg->lookahead_sz2 = DEF_LOOKAHEAD_SZ2;
    cfg->buffer_size = DEF_BUFFER_SIZE;
    cfg->decoder_input_buffer_size = DEF_DECODER_INPUT_BUFFER_SIZE;
    cfg->cmd = OP_ENC;
    cfg->verbose = 0;
    cfg->in_fname = "-";
    cfg->out_fname = "-";

    printf("Start processing %d arguments\n", argc);

    int a = 0;
    while ((a = getopt(argc, argv, "hedi:w:l:v")) != -1) {
    	printf("Parsing argument @ %d = '%c'\n", optind, a);
        switch (a) {
        case 'h':               /* help */
            usage();
            /* FALLTHROUGH */
        case 'e':               /* encode */
            cfg->cmd = OP_ENC; break;
        case 'd':               /* decode */
            cfg->cmd = OP_DEC; break;
        case 'i':               /* input buffer size */
        	cfg->decoder_input_buffer_size = atoi(optarg);
            //cfg->buffer_size = atoi(optarg);
            break;
        case 'w':               /* window bits */
            cfg->window_sz2 = atoi(optarg);
            break;
        case 'l':               /* lookahead bits */
            cfg->lookahead_sz2 = atoi(optarg);
            break;
        case 'v':               /* verbosity++ */
            cfg->verbose++;
            break;
        case '?':               /* unknown argument */
        default:
            usage();
        }
    }
    argc -= optind;
    argv += optind;
    if (argc > 0) {
        cfg->in_fname = argv[0];
        argc--;
        argv++;
    }
    if (argc > 0) { cfg->out_fname = argv[0]; }
}

int main(int argc, char **argv) {
#if 1
    if(argc == 1)
    {
    	fprintf(stderr, "Input file not specified - please provide a file as the first parameter!\n");
    	exit(1);
    }
	/* This path through the code forces a set of known values to keep the
	 * CLI simple for testing. */
	const uint8_t window_size_bits = 11;
	const uint8_t lookahead_size_bits = 4;
    config cfg =
	{
			.window_sz2	= window_size_bits,
			.lookahead_sz2 = lookahead_size_bits,
			.buffer_size = (1 << window_size_bits),
			.decoder_input_buffer_size = 256,
			.cmd = OP_ENC,
			.verbose = 1, // Enable verbosity for testing.
			.in_fname = argv[1],
	};
    const size_t out_file_len = strlen(argv[1]) + 14; /* 4 digits for window_size and lookahead size, then .hs and underscores etc... */

    char out_fname[out_file_len];

    if(argv[argc -1][0] == 'd')
    {
    	cfg.cmd = OP_DEC;

        snprintf(out_fname, out_file_len, "%s_w%2d_l%d.out", argv[1], window_size_bits,  lookahead_size_bits);
    }
    else
    {
        snprintf(out_fname, out_file_len, "%s_w%2d_l%d.hs", argv[1], window_size_bits,  lookahead_size_bits);
    }
    cfg.out_fname = out_fname;

#else
    config cfg;
    memset(&cfg, 0, sizeof(cfg));
    proc_args(&cfg, argc, argv);

    if (0 == strcmp(cfg.in_fname, cfg.out_fname)
        && (0 != strcmp("-", cfg.in_fname))) {
        fprintf(stderr, "Refusing to overwrite file '%s' with itself.\n", cfg.in_fname);
        exit(1);
    }
#endif

    cfg.in = handle_open(cfg.in_fname, IO_READ, cfg.buffer_size);
    if (cfg.in == NULL) { die("Failed to open input file for read"); }
    cfg.out = handle_open(cfg.out_fname, IO_WRITE, cfg.buffer_size);
    if (cfg.out == NULL) { die("Failed to open output file for write"); }

#if _WIN32
    /*
     * On Windows, stdin and stdout default to text mode. Switch them to
     * binary mode before sending data through them.
     */
    _setmode(STDOUT_FILENO, O_BINARY);
    _setmode(STDIN_FILENO, O_BINARY);
#endif
    printf("Config input from \"%s\" output to \"%s\" \n", cfg.in_fname, cfg.out_fname);
    printf("Window = %d bytes, Lookahead %d bytes, input Buffer %ld\n",  (1<<cfg.window_sz2), (1<< cfg.lookahead_sz2), cfg.buffer_size);
    cfg.verbose = 1;

    if (cfg.cmd == OP_ENC) {
    	printf("Start Encoding...\n");
    	return encode(&cfg);
    } else if (cfg.cmd == OP_DEC) {
    	printf("Start Decoding...\n");
        return decode(&cfg);
    } else {
        usage();
    }
}
