/*
 * DIAG MTS for LGE MTS Kernel Driver
 *
 *  lg-msp TEAM <lg-msp@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
*/
#include "mtsk_tty.h"
#ifdef CONFIG_LGE_MTS
#define HDLC_END 0x7E
#define _BUF_SIZE_MTSK_16K 16384
#define _BUF_SIZE_MTSK (_BUF_SIZE_MTSK_16K + 1024)

inline static int mtsk_tty_msg_type (char cmd) {
    if(cmd == DIAG_EXT_MSG_F || cmd == DIAG_QSR_EXT_MSG_TERSE_F)
        return TRUE;

    return FALSE;
}

void mtsk_tty_busy_unlock(char *buf) {
    if (buf == (void *)driver->buf_in_1 && driver->ch) {
        driver->in_busy_1 = 0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
    }
    else if (buf == (void *)driver->buf_in_2 && driver->ch) {
        driver->in_busy_2 =0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
    }
    else if (buf == (void *)driver->buf_in_qdsp_1 && driver->chqdsp) {
        driver->in_busy_qdsp_1 =0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
    }
    else if (buf == (void *)driver->buf_in_qdsp_2 && driver->chqdsp) {
        driver->in_busy_qdsp_2 =0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work)); 
    }
#ifdef CONFIG_DIAG_HSIC_PIPE
    else if (buf == (void *)driver->buf_in_wcnss_1 && driver->ch_wcnss) {
        driver->in_busy_wcnss_1 = 0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_wcnss_work));
    }
    else if (buf == (void *)driver->buf_in_wcnss_2 && driver->ch_wcnss) {
        driver->in_busy_wcnss_2 = 0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_wcnss_work));
    }
    else if (buf == (void *)driver->buf_in_hsic && driver->hsic_ch) {
        driver->in_busy_hsic_write_on_device = 0;
        queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
    }
#else
    else if (buf == (void *)driver->buf_in_wcnss && driver->ch_wcnss) {
        driver->in_busy_wcnss = 0;
        queue_work(driver->diag_wq, &(driver->diag_read_smd_wcnss_work));
    }
#endif
#ifdef CONFIG_DIAG_SDIO_PIPE
    else if (buf == (void *)driver->buf_in_sdio && driver->sdio_ch) {
        driver->in_busy_sdio = 0;
        queue_work(driver->diag_sdio_wq, &(driver->diag_read_sdio_work));
    }
#endif
#ifdef CONFIG_DIAG_HSIC_PIPE
    else if (buf == (void *)driver->buf_in_hsic && driver->hsic_ch) {
        driver->in_busy_hsic_write_on_device = 0;
        queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
    }
#endif
}

inline int mtsk_tty_msg_prev (char* b, int len, int prev_item, int* buf_pos) {
    int loop = 0;

    for (loop = 0; loop < len; loop++) {
        if(b[loop] == HDLC_END)
            break;
    }

    if(prev_item == 2)  /* DIAG CMD */
        *buf_pos = loop + 1;
    else 
        *buf_pos = 0;

    return loop + 1;
}

inline int mtsk_tty_msg_main (char* buf, int len, char* b, int buf_pos, int loop_start, int* remind) {
    int loop = 0;
    int pos = 0;

    for (loop = loop_start  ; loop < len; loop++) {
        buf[buf_pos + pos] = b[loop];

        if (buf[buf_pos + pos] == HDLC_END) {
            if(mtsk_tty_msg_type(buf[buf_pos]) == FALSE) /*DIAG CMD */
                buf_pos = buf_pos + pos + 1;
            pos = 0;
        }
        else
            pos++;
    }

    *remind = pos;
    return buf_pos;
}

int mtsk_tty_msg (char* buf, int len, int proc_num) {
    static int prev_item = 0; /* 0 : not , 1: DIAG MSG , 2: DIAG CMD */
    static char* b = NULL;
    int count = 0;
    int loop_start = 0;
    int buf_pos = 0;

    int remind = 0;

    if(len > _BUF_SIZE_MTSK_16K)
        return len;

    if(b == NULL) {
        b = kzalloc(_BUF_SIZE_MTSK, GFP_KERNEL);

        if(b == NULL)
            goto _mem_fail;
    }

    switch (proc_num) {
        case MODEM_DATA:
#ifdef CONFIG_DIAG_SDIO_PIPE
        case SDIO_DATA:
#endif
#ifdef CONFIG_DIAG_HSIC_PIPE
        case HSIC_DATA:
#endif
            break;

        default :
            goto _bypass;
            break;
    }

    memcpy (b, buf , len);

    if (prev_item != 0)
        loop_start = mtsk_tty_msg_prev(b, len, prev_item, &buf_pos);

    count =  mtsk_tty_msg_main (buf, len, b, buf_pos, loop_start, &remind);
    prev_item = 0;

    if(remind > 0) {
        if(mtsk_tty_msg_type(buf[count]) == FALSE) { /* DIAG CMD */
            count = count + remind;
            prev_item = 2;
        }
        else /* DIAG MSG */
            prev_item = 1;
    }

    mtsk_tty_push(b ,len);

    if(count == 0)
        mtsk_tty_busy_unlock(buf);

    return count;

_bypass:
    mtsk_tty_push(buf,len);
    mtsk_tty_busy_unlock(buf);
    return 0;

_mem_fail: // MEMORY Allocation fail ....
    if(b != NULL) 
        kfree(b);

    return len;
}

int mtsk_tty_process (char * buf, int* plen, int proc_num) {
    int len = *plen;
    if (init_mtsk_tty == FALSE)
        return TRUE;

    if(MTS_TTY_OPEN != mtsk_tty->tty_state)
        return TRUE;

    *plen = mtsk_tty_msg(buf ,len, proc_num);

    return *plen;
}
#endif
