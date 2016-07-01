/*
 * Author: Joseph DelPreto
 */

#include "ToneDetector.h"

#ifdef acousticControl

// The static instance
ToneDetector toneDetector;
#ifdef streamAcousticControlLog
int32_t acousticControlLogToStream[5] = {0,0,0,0,0};
#endif

//============================================
// Initialization
//============================================

// Constructor
ToneDetector::ToneDetector() :
    terminated(0),
    fillingBuffer0(false),
    transferComplete(false),
    readyToBegin(false),
    callbackFunction(0)
    #ifdef artificialSamplesMode
    , numTestSamples(0),
    testSampleIndex(0),
    sampleIndex(0)
    #endif
{
}

// Set a callback function that should be called after each buffer is processed
void ToneDetector::setCallback(void (*myFunction)(int32_t* tonePowers, uint32_t signalLevel))
{
    callbackFunction = myFunction;
}

// Initialize the Goertzel variables, arrays, etc.
// sampleWindow, sampleInterval, and tones must have been previously set
//   if not, this method will return without doing anything
void ToneDetector::init()
{
    // Make sure sampleWindow, sampleInterval, and desired tones have been set
    if(sampleWindow == 0 || sampleInterval == 0 || numTones == 0)
        return;

    // Initialize sample buffers
    //printf("Sampling interval: %d us\n", sampleInterval);
    //printf("Buffer size: %d samples\n", sampleWindow);
    for(uint16_t i = 0; i < sampleWindow; i++) // not using memset since sizeof seems to not work with uint16_t?
    {
        sampleBuffer0[i] = 0;
        sampleBuffer1[i] = 0;
    }
    #if defined(recordSamples) && !defined(recordStreaming)
    savedSamplesIndex = 0;
    for(uint16_t i = 0; i < numSavedSamples; i++) // not using memset since sizeof seems to not work with uint16_t?
        savedSamples[i] = 0;
    #endif

    // Initialize goertzel arrays
    tonePowersWindowIndex = 0;
    for(int i = 0; i < numTones; i++)
    {
        goertzelCoefficients[i] = 0;
        for(int w = 0; w < tonePowersWindow; w++)
            tonePowers[i][w] = 0;
        tonePowersSum[i] = 0;
    }
    readyToThreshold = false;

    // Some convenience variables as doubles for precision (will cast to fixed point later)
    double sampleFrequency = 1000.0/((double)sampleInterval/1000.0); // Hz
    double N = (double) sampleWindow;

    // Calculate the coefficient for each tone
    //printf("Initializing Goertzel algorithm\n");
    //printf("  Bin size: %f\n", sampleFrequency/N);
    for(int i = 0; i < numTones; i++)
    {
        // Determine K and then f for desired tone (f = desiredF/Fs)
        // tone/fs = f = K/N --> K = (int)(tone*N/fs)
        double tone = targetTones[i];
        int k = (int) (0.5 + ((N * tone) / sampleFrequency));
        float f = ((float)k) / N;
        //printf("  Desired tone %f -> %f", tone, f*sampleFrequency);
        float cosine = cos(2.0 * PI * f);
        // Store the result as a fixed-point number
        goertzelCoefficients[i] = toFixedPoint(2.0*cosine);
        //printf("\t  (coefficient: %f)\n", toFloat(goertzelCoefficients[i]));
    }

    #if defined(recordOutput) && !defined(recordStreaming)
    savedTonePowersIndex = 0;
    for(uint16_t i = 0; i < numSavedTonePowers; i++)    // not using memset since sizeof seems to not work with uint16_t?
    {
        for(uint8_t t = 0; t < numTones; t++)
            savedTonePowers[i][t] = 0;
    }
    #endif

    // We're ready to process some tunes!
    readyToBegin = true;
}

#ifndef artificialSamplesMode
// Configure and start the DMA channels for ADC sample gathering
// Will have a linked list of two DMA operations, one for each buffer
void ToneDetector::startDMA()
{
    // Create the Linked List Items
    lli[0] = new MODDMA_LLI;
    lli[1] = new MODDMA_LLI;

    // Prepare DMA configuration, which will be chained (one operation for each buffer)
    dmaConf = new MODDMA_Config;
    dmaConf
     ->channelNum    ( MODDMA::Channel_0 )
     ->srcMemAddr    ( 0 )
     ->dstMemAddr    ( (uint32_t)sampleBuffer0 )
     ->transferSize  ( sampleWindow )
     ->transferType  ( MODDMA::p2m )
     ->transferWidth ( MODDMA::word )
     ->srcConn       ( MODDMA::ADC )
     ->dstConn       ( 0 )
     ->dmaLLI        ( (uint32_t)lli[1] ) // Looks like it does the above setup and then calls this LLI - thus we have this setup mimic lli[0] and then the chain actually starts by calling lli[1]
     ->attach_tc     ( &TC0_callback )
     ->attach_err    ( &ERR0_callback )
    ;
    // Create LLI to transfer from ADC to adcBuffer0 (and then launch lli[1])
    lli[0]->SrcAddr = (uint32_t)dma.LUTPerAddr(dmaConf->srcConn());
    lli[0]->DstAddr = (uint32_t)sampleBuffer0;
    lli[0]->NextLLI = (uint32_t) lli[1];
    lli[0]->Control = dma.CxControl_TransferSize(dmaConf->transferSize())
                | dma.CxControl_SBSize((uint32_t)dma.LUTPerBurst(dmaConf->srcConn()))
                | dma.CxControl_DBSize((uint32_t)dma.LUTPerBurst(dmaConf->srcConn()))
                | dma.CxControl_SWidth((uint32_t)dma.LUTPerWid(dmaConf->srcConn()))
                | dma.CxControl_DWidth((uint32_t)dma.LUTPerWid(dmaConf->srcConn()))
                | dma.CxControl_DI()
                | dma.CxControl_I();
    // Create LLI to transfer from ADC to adcBuffer1 (and then launch lli[0] to repeat)
    lli[1]->SrcAddr = (uint32_t)dma.LUTPerAddr(dmaConf->srcConn());
    lli[1]->DstAddr = (uint32_t)sampleBuffer1;
    lli[1]->NextLLI = (uint32_t) lli[0];
    lli[1]->Control = dma.CxControl_TransferSize(dmaConf->transferSize())
                | dma.CxControl_SBSize((uint32_t)dma.LUTPerBurst(dmaConf->srcConn()))
                | dma.CxControl_DBSize((uint32_t)dma.LUTPerBurst(dmaConf->srcConn()))
                | dma.CxControl_SWidth((uint32_t)dma.LUTPerWid(dmaConf->srcConn()))
                | dma.CxControl_DWidth((uint32_t)dma.LUTPerWid(dmaConf->srcConn()))
                | dma.CxControl_DI()
                | dma.CxControl_I();

    // Start the DMA chain
    fillingBuffer0 = true;
    transferComplete = false;
    if (!dma.Prepare(dmaConf)) {
        error("Doh! Error preparing initial dma configuration");
    }
}

// Configure the ADC to trigger on Timer1
// Start the timer with the desired sampling interval
void ToneDetector::startADC()
{
    // We use the ADC irq to trigger DMA and the manual says
    // that in this case the NVIC for ADC must be disabled.
    NVIC_DisableIRQ(ADC_IRQn);

    // Power up the ADC and set PCLK
    LPC_SC->PCONP    |=  (1UL << 12); // enable power
    LPC_SC->PCLKSEL0 &= ~(3UL << 24); // Clear divider to use CCLK/8 = 12MHz directly (see page 57) // original example code comment: PCLK = CCLK/4 96M/4 = 24MHz

    // Enable the ADC, 12MHz,  ADC0.0
    LPC_ADC->ADCR  = (1UL << 21);
    LPC_ADC->ADCR &= ~(255 << 8); // No clock divider (use the 12MHz directly)

    // Set the pin functions to ADC (use pin p15)
    LPC_PINCON->PINSEL1 &= ~(3UL << 14);  /* P0.23, Mbed p15. */
    LPC_PINCON->PINSEL1 |=  (1UL << 14);

    // Enable ADC irq flag (to DMA).
    // Note, don't set the individual flags,
    // just set the global flag.
    LPC_ADC->ADINTEN = 0x100;

    // (see page 586 of http://www.nxp.com/documents/user_manual/UM10360.pdf)
    // Disable burst mode
    LPC_ADC->ADCR &= ~(1 << 16);
    // Have the ADC convert based on timer 1
    LPC_ADC->ADCR |= (6 << 24); // Trigger on MAT1.0
    LPC_ADC->ADCR |= (1 << 27); // Falling edge

    // Set up timer 1
    LPC_SC->PCONP    |= (1UL << 2);          // Power on Timer1
    LPC_SC->PCLKSEL0 &= ~(3 << 4);           // No clock divider (use 12MHz directly) (see page 57 of datasheet)
    LPC_TIM1->PR      = 11;                  // TC clocks at 1MHz since we selected 12MHz above (see page 507 of datasheet)
    LPC_TIM1->MR0     = sampleInterval-1;    // sampling interval in us
    LPC_TIM1->MCR     = 3;                   // Reset TCR to zero on match
    LPC_TIM1->EMR     = (3UL<<4)|1;          // Make MAT1.0 toggle.
    //NVIC_EnableIRQ(TIMER1_IRQn);           // Enable timer1 interrupt NOTE: enabling the interrupt when MCR is 3 will make everything stop working.  enabling the interrupt when MCR is 2 will work but the interrupt isn't actually called.

    // Start the timer (which thus starts the ADC)
    LPC_TIM1->TCR=0;
    LPC_TIM1->TCR=1;
}
#endif // end if(not artificial samples mode)

//============================================
// Execution Control
//============================================

// Start acquiring and processing samples
// Will run forever or until stop() is called
void ToneDetector::run()
{
    if(!readyToBegin)
        return;
    terminated = false;
    //printf("\nTone detector starting...\n");
    #ifdef recordStreaming
        #ifdef recordSamples
        printf("\tSample (V)");
        printf("\n");
        #endif
        #ifdef recordOutput
        for(uint8_t t = 0; t < numTones; t++)
            printf("\t%f Hz", targetTones[t]);
        printf("\n");
        #endif
    #endif

    // Set up initial buffer configuration
    samplesWriting = sampleBuffer0;
    samplesProcessing = sampleBuffer1;
    fillingBuffer0 = true;
    transferComplete = false;
    // Start periodically sampling
    #ifdef artificialSamplesMode
    initTestModeSamples(); // artificially create samples
    sampleTicker.attach_us(&toneDetector, &ToneDetector::tickerCallback, sampleInterval);  // "sample" artificial samples at desired rate
    #else
    startDMA();
    if(!terminated) // If DMA got an error, terminated will be set true
        startADC();
    #endif

    #ifdef debugLEDs
    led1 = 1; // Indicate start of tone detection
    #endif
    #ifdef debugPins // after LEDs to be more accurate
    debugPin1 = 1;   // Indicate start of tone detection
    #endif

    // Main loop
    // Wait for buffers to fill and then process them
    while(!terminated)
    {
        // Check if a buffer of samples is gathered and if so process it
        if(transferComplete)
        {
            transferComplete = false;
            processSamples();
        }
    }

    #ifdef debugPins // before LEDs to be more accurate
    debugPin1 = 0; // Indicate cessation of tone detection
    debugPin2 = 0; // Turn off indicator that at least two buffers were processed
    debugPin4 = 1; // Indicate completion
    #endif
    #ifdef debugLEDs
    led1 = 0; // Indicate cessation of tone detection
    led2 = 0; // Turn off indicator that at least one buffer was processed
    led4 = 1; // Indicate completion
    #endif
}

// Finish up (write results to file and whatnot)
// This is separate method so that main program can time the running itself without this extra overhead
void ToneDetector::finish()
{
    //printf("Tone detector finished\n");

    #if defined(recordSamples) && !defined(recordStreaming)
        // Write saved samples to file
        LocalFileSystem local("local");
        FILE *foutSamples = fopen("/local/samples.wp", "w");  // Open "samples.wp" on the local file system for writing
        fprintf(foutSamples, "Sample (V)\n");
        uint16_t savedSamplesN = savedSamplesIndex;
        do
        {
            fprintf(foutSamples, "%f\n", toFloat(savedSamples[savedSamplesN])/4.0*3.3);
            savedSamplesN++;
            savedSamplesN %= numSavedSamples;
        } while(savedSamplesN != savedSamplesIndex);
        fclose(foutSamples);
    #endif // recordSamples && !recordStreaming
    #if defined(recordOutput) && !defined(recordStreaming)
        // Write saved outputs to file
        #ifndef recordSamples
        LocalFileSystem local("local");
        #endif // not recordSamples
        FILE *foutOutput = fopen("/local/out.wp", "w");  // Open "out.wp" on the local file system for writing
        for(uint8_t t = 0; t < numTones; t++)
            fprintf(foutOutput, "%f Hz\t", tones[t]);
        uint16_t savedTonePowersN = savedTonePowersIndex;
        do
        {
            for(uint8_t t = 0; t < numTones; t++)
                fprintf(foutOutput, "%ld  \t", savedTonePowers[savedTonePowersN][t]);
            fprintf(foutOutput, "\n");
            savedTonePowersN++;
            savedTonePowersN %= numSavedTonePowers;
        } while(savedTonePowersN != savedTonePowersIndex);
        fclose(foutOutput);
    #endif // recordOutput
}

// Terminate the tone detector
// Note: Will actually terminate after next time buffer1 is filled, so won't be instantaneous
void ToneDetector::stop()
{
    // Stop sampling
    #ifdef artificialSamplesMode
    sampleTicker.detach();
    #else
    lli[1]->Control = 0;                        // Make the DMA stop after next time buffer1 is filled
    while(!(!fillingBuffer0 && transferComplete)); // Wait for buffer1 to be filled
    LPC_TIM1->TCR=0;              // Stop the timer (and thus the ADC)
    LPC_SC->PCONP &= ~(1UL << 2); // Power off the timer
    #endif

    // Stop the main loop
    terminated = true;
}

//============================================
// Sampling / Processing
//============================================

// Acquire a new sample
#ifdef artificialSamplesMode
// If a buffer has been filled, swap buffers and signal the main thread to process it
void ToneDetector::tickerCallback()
{
    // Get a sample
    samplesWriting[sampleIndex] = testSamples[testSampleIndex];
    testSampleIndex++;
    testSampleIndex %= numTestSamples;

    // Increment sample index
    sampleIndex++;
    sampleIndex %= sampleWindow;

    // See if we just finished a buffer
    if(sampleIndex == 0)
    {
        // Swap writing and processing buffers
        // Let the main tone detector thread know that processing should take place
        if(fillingBuffer0)
        {
            samplesProcessing = sampleBuffer0;
            samplesWriting = sampleBuffer1;
        }
        else
        {
            samplesProcessing = sampleBuffer1;
            samplesWriting = sampleBuffer0;
        }
        transferComplete = true;
        fillingBuffer0 = !fillingBuffer0;
    }
}
#else // not artificial mode - we want real samples!
// Callback for DMA channel 0
void TC0_callback(void)  // static method
{
    // Swap writing and processing buffers used by main loop
    if(toneDetector.fillingBuffer0)
    {
        toneDetector.samplesProcessing = toneDetector.sampleBuffer0;
        toneDetector.samplesWriting = toneDetector.sampleBuffer1;
    }
    else
    {
        toneDetector.samplesProcessing = toneDetector.sampleBuffer1;
        toneDetector.samplesWriting = toneDetector.sampleBuffer0;
    }
    // Tell main() loop that this buffer is ready for processing
    toneDetector.fillingBuffer0 = !toneDetector.fillingBuffer0;
    toneDetector.transferComplete = true;

    // Clear DMA IRQ flags.
    if(toneDetector.dma.irqType() == MODDMA::TcIrq) toneDetector.dma.clearTcIrq();
    if(toneDetector.dma.irqType() == MODDMA::ErrIrq) toneDetector.dma.clearErrIrq();
}

// Configuration callback on Error for channel 0
void ERR0_callback(void)  // static method
{
    // Stop sampling
    LPC_TIM1->TCR = 0;            // Stop the timer (and thus the ADC)
    LPC_SC->PCONP &= ~(1UL << 2); // Power off the timer

    // Stop the main loop (don't call stop() since that would wait for next buffer to fill)
    toneDetector.terminated = true;

    error("Oh no! My Mbed EXPLODED! :( Only kidding, go find the problem (DMA chan 0)");
}
#endif

// Goertzelize the process buffer
void ToneDetector::processSamples()
{
    #ifdef debugLEDs
    if(fillingBuffer0)
        led2 = 1; // Indicate that at least two buffers have been recorded
    led3 = 1;     // Indicate start of processing
    #endif
    #ifdef debugPins // after LEDs and timer to be more accurate
    if(fillingBuffer0)
        debugPin2 = 1; // Indicate that at least two buffers have been recorded
    debugPin3 = 1;     // Indicate start of processing
    #endif

    // Create variables for storing the Goertzel series
    int32_t s0, s1, s2;
    volatile int32_t newTonePower;
    // Create variables for getting max input value
    int32_t sample = 0;
    uint32_t signalLevel = 0;
    // For each desired tone, compute power and then reset the Goertzel
    for(uint8_t i = 0; i < numTones; i++)
    {
        // Reset
        s0 = 0;
        s1 = 0;
        s2 = 0;
        // Compute the Goertzel series
        for(uint16_t n = 0; n < sampleWindow; n++)
        {
            // Note: bottom 4 bits from ADC indicate channel, top 12 are the actual data
            // Note: Assuming Q10 format, we are automatically scaling the ADC value to [0, 4] range
            sample = ((int32_t)((samplesProcessing[n] >> 4) & 0xFFF));
            // Get signal level indicator
            if(i == 0)
            {
                if(sample-2048 >= 0)
                    signalLevel += (sample-2048);
                else
                    signalLevel += (2048-sample);
            }
            // TODO check the effect of this subtraction?
            //sample -= 2048;
            s0 = sample + fixedPointMultiply(goertzelCoefficients[i], s1) - s2;
            s2 = s1;
            s1 = s0;
        }
        // Compute the power
        newTonePower = fixedPointMultiply(s2,s2) + fixedPointMultiply(s1,s1) - fixedPointMultiply(fixedPointMultiply(goertzelCoefficients[i],s1),s2);
        // Update the running sum
        tonePowersSum[i] -= tonePowers[i][tonePowersWindowIndex];
        tonePowersSum[i] += newTonePower;
        // Update the history of powers
        tonePowers[i][tonePowersWindowIndex] = newTonePower;
    }
    // See if first circular buffer has been filled
    readyToThreshold = readyToThreshold || (tonePowersWindowIndex == tonePowersWindow-1);
    // Deliver results if a callback function was provided
    if(callbackFunction != 0 && readyToThreshold)
        callbackFunction(tonePowersSum, signalLevel >> 7); // divide signal level by 128 is basically making it an average (125 samples/buffer)
	#ifdef streamAcousticControlLog
	acousticControlLogToStream[0] = tonePowers[0][tonePowersWindowIndex];
	acousticControlLogToStream[1] = tonePowers[1][tonePowersWindowIndex];
	acousticControlLogToStream[2] = signalLevel >> 7;
	#endif
    #ifdef debugLEDs
    led3 = 0;        // Indicate completion of processing
    #endif
    #ifdef recordSamples
        #ifdef recordStreaming
        for(uint16_t n = 0; n < sampleWindow; n++)
            printf("%ld\n", (samplesProcessing[n] >> 4) & 0xFFF);
        #else
        for(uint16_t n = 0; n < sampleWindow; n++)
        {
            savedSamples[savedSamplesIndex++] = (samplesProcessing[n] >> 4) & 0xFFF;
            savedSamplesIndex %= numSavedSamples;
        }
        #endif
    #endif
    #ifdef recordOutput
        #ifdef recordStreaming
        for(uint8_t t = 0; t < numTones; t++)
            printf("%ld\t", tonePowers[t][tonePowersWindowIndex] >> 10); // used to shift 10
        printf("\n");
        #else
        for(uint8_t t = 0; t < numTones; t++)
        {
            savedTonePowers[savedTonePowersIndex][t] = tonePowers[t][tonePowersWindowIndex];
        }
        savedTonePowersIndex++;
        savedTonePowersIndex %= numSavedTonePowers;
        #endif
    #endif
    // Increment window index (circularly)
    tonePowersWindowIndex = (tonePowersWindowIndex+1) % tonePowersWindow;

    #ifdef debugPins // before LEDs, timer, and recordSamples to be more accurate
    debugPin3 = 0;   // Indicate completion of processing
    #endif
}

int32_t* ToneDetector::getTonePowers()
{
    return tonePowersSum;
}

//============================================
// Testing / Debugging
//============================================

#ifdef artificialSamplesMode
// Use artificial samples, either from a file or from summing cosine waves of given frequencies
// Samples in a file will be interpreted as volts, so should be in range [0, 3.3]
void ToneDetector::initTestModeSamples()
{
    #ifdef sampleFilename
    LocalFileSystem local("local");
    printf("Using samples from file: "); printf(sampleFilename); printf("\n");
    FILE *fin = fopen(sampleFilename, "r");  // Open "setup.txt" on the local file system for read
    if(fin == 0)
    {
        printf("  *** Cannot open file! ***\n");
        wait_ms(3000);
        numTestSamples = 1;
        testSamples = new uint32_t[numTestSamples];
        testSamples[0] = 0;
        testSampleIndex = 0;
        return;
    }
    // See how long the file is
    int numTestSamples = 0;
    float val;
    float maxVal = 0;
    float minVal = 0;
    float avgVal = 0;
    int res = fscanf(fin, "%d\n", &val);
    while(res > 0)
    {
        numTestSamples++;
        res = fscanf(fin, "%f\n", &val);
        if(val > maxVal)
            maxVal = val;
        if(val < minVal)
            minVal = val;
        avgVal = numTestSamples > 1 ? (avgVal + val)/2.0 : val;
    }
    printf("  Found %d samples in the file, max %f, min %f, avg %f\n", numTestSamples, maxVal, minVal, avgVal);
    if(minVal < 0)
        printf("  WARNING: File supposed to represent voltage input to ADC, so negative numbers will be interpreted as 0\n");
    if(maxVal > 3.3)
        printf("  WARNING: File supposed to represent voltage input to ADC, so numbers greater than 3.3 will be interpreted as 3.3\n");
    fclose(fin);
    // Read the samples
    testSamples = new uint32_t[numTestSamples];
    fin = fopen(sampleFilename, "r");  // Open "setup.txt" on the local file system for read
    for(int i = 0; i < numTestSamples; i++)
    {
        // Read the voltage
        fscanf(fin, "%f\n", &val);
        // Clip it like the ADC would
        val = val > 3.3 ? 3.3 : val;
        val = val < 0 ? 0 : val;
        // Convert voltage to 12-bit ADC reading
        testSamples[i] = val/3.3*4096.0;
        // Shift it by 4 to mimic what the DMA would write for a reading (lower 4 would indicate ADC channel)
        testSamples[i] = testSamples[i] << 4;
    }
    fclose(fin);
    testSampleIndex = 0;
    sampleIndex = 0;

    #else // not using file for samples, will create a sum of cosine waves instead

    numTestSamples = 1000;
    testSamples = new uint32_t[numTestSamples];
    testSampleIndex = 0;

    // Adjust overall amplitude and offset - make sure it's nonnegative
    float amplitude = 1;  // volts
    float baseline = 1.5; // volts
    // Print out the frequencies being used
    float frequencies[] = sumSampleFrequencies; // Test signal will be a summation of cosines at these frequencies (in Hz)
    printf("Using samples from cosines with the following frequencies:\n");
    for(int f = 0; f < sizeof(frequencies)/sizeof(float); f++)
        printf("  %f\n", frequencies[f]);
    printf("\n");

    // Create the samples
    float sampleFrequency = 1000.0/((double)sampleInterval/1000.0);
    for(uint16_t n = 0; n < numTestSamples; n++)
    {
        float nextSample = 0;
        // Sum the frequencies
        for(int f = 0; f < sizeof(frequencies)/sizeof(float); f++)
            nextSample += cos(2.0*PI*frequencies[f]*(float)n/sampleFrequency);
        // Normalize
        nextSample /= (float)(sizeof(frequencies)/sizeof(float));
        // Amplify
        nextSample *= amplitude;
        // Positivify
        nextSample += baseline;
        // Convert to 12-bit ADC reading
        testSamples[n] = ((uint32_t)(nextSample/3.3*4095.0));
        // Shift it by 4 to mimic what the DMA would write for a reading (lower 4 would indicate ADC channel)
        testSamples[n] = testSamples[n] << 4;
    }
    sampleIndex = 0;
    #endif // which sample mode
}
#endif // artificialSamplesMode

#endif // #ifdef acousticControl
