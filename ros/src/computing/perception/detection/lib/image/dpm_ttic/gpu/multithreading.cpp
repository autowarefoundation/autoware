/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.
 * Any use, reproduction, disclosure, or distribution of this software
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA)
 * associated with this source code for terms and conditions that govern
 * your use of this NVIDIA software.
 *
 */

#include "multithreading.h"

CUTThread cutStartThread(CUT_THREADROUTINE func, void * data)
{
        pthread_t thread;
        pthread_create(&thread, NULL, func, data);
        return thread;
}

void cutEndThread(CUTThread thread)
{
        pthread_join(thread, NULL);
}

void cutDestroyThread(CUTThread thread)
{
        pthread_cancel(thread);
}

void cutWaitForThreads(const CUTThread * threads, int num)
{
        for(int i = 0; i < num; i++)
		cutEndThread(threads[i]);
}
