/*******************************************************************************
* File Name: USER_PORT_I2C_INT.c
* Version 2.0
*
* Description:
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in I2C mode.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USER_PORT_PVT.h"
#include "USER_PORT_I2C_PVT.h"


/*******************************************************************************
* Function Name: USER_PORT_I2C_ISR
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the SCB I2C mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
CY_ISR(USER_PORT_I2C_ISR)
{
    uint32 diffCount;
    uint32 endTransfer;

    #if(USER_PORT_CHECK_I2C_ACCEPT_ADDRESS_CONST)
        uint32 address;
    #endif /* (USER_PORT_CHECK_I2C_ACCEPT_ADDRESS_CONST) */

    endTransfer = 0u; /* Continue active transfer */

    /* Calls customer routine if registered */
    if(NULL != USER_PORT_customIntrHandler)
    {
        USER_PORT_customIntrHandler();
    }

    if(USER_PORT_CHECK_INTR_I2C_EC_MASKED(USER_PORT_INTR_I2C_EC_WAKE_UP))
    {
        /* Mask-off after wakeup */
        USER_PORT_SetI2CExtClkInterruptMode(USER_PORT_NO_INTR_SOURCES);
    }

    /* Master and Slave error tracking:
    * Add the master state check to track only the master errors when the master is active or
    * track slave errors when the slave is active or idle.
    * A special MMS case: in the address phase with misplaced Start: the master sets the LOST_ARB and
    * slave BUS_ERR. The valid event is LOST_ARB comes from the master.
    */
    if(USER_PORT_CHECK_I2C_FSM_MASTER)
    {
        if(USER_PORT_I2C_MASTER)
        {
            /* INTR_MASTER_I2C_BUS_ERROR:
            * A misplaced Start or Stop condition occurred on the bus: complete the transaction.
            * The interrupt is cleared in I2C_FSM_EXIT_IDLE.
            */
            if(USER_PORT_CHECK_INTR_MASTER_MASKED(USER_PORT_INTR_MASTER_I2C_BUS_ERROR))
            {
                USER_PORT_mstrStatus |= (uint16) (USER_PORT_I2C_MSTAT_ERR_XFER |
                                                         USER_PORT_I2C_MSTAT_ERR_BUS_ERROR);

                endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
            }

            /* INTR_MASTER_I2C_ARB_LOST:
            * The MultiMaster lost arbitrage during transaction.
            * A Misplaced Start or Stop condition is treated as lost arbitration when the master drives the SDA.
            * The interrupt source is cleared in I2C_FSM_EXIT_IDLE.
            */
            if(USER_PORT_CHECK_INTR_MASTER_MASKED(USER_PORT_INTR_MASTER_I2C_ARB_LOST))
            {
                USER_PORT_mstrStatus |= (uint16) (USER_PORT_I2C_MSTAT_ERR_XFER |
                                                         USER_PORT_I2C_MSTAT_ERR_ARB_LOST);

                endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
            }

            if(USER_PORT_I2C_MULTI_MASTER_SLAVE)
            {
                /* I2C_MASTER_CMD_M_START_ON_IDLE:
                * MultiMaster-Slave does not generate start, because Slave was addressed.
                * Pass control to slave.
                */
                if(USER_PORT_CHECK_I2C_MASTER_CMD(USER_PORT_I2C_MASTER_CMD_M_START_ON_IDLE))
                {
                    USER_PORT_mstrStatus |= (uint16) (USER_PORT_I2C_MSTAT_ERR_XFER |
                                                             USER_PORT_I2C_MSTAT_ERR_ABORT_XFER);

                    endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                }
            }
            

            /* The error handling common part:
            * Sets a completion flag of the master transaction and passes control to:
            *  - I2C_FSM_EXIT_IDLE - to complete transaction in case of: ARB_LOST or BUS_ERR.
            *  - I2C_FSM_IDLE      - to take chance for the slave to process incoming transaction.
            */
            if(0u != endTransfer)
            {
                /* Set completion flags for master */
                USER_PORT_mstrStatus |= (uint16) USER_PORT_GET_I2C_MSTAT_CMPLT;

                if(USER_PORT_I2C_MULTI_MASTER_SLAVE)
                {
                    if(USER_PORT_CHECK_I2C_FSM_ADDR)
                    {
                        /* Start generation is set after another master starts accessing Slave.
                        * Clean-up master and turn to slave. Set state to IDLE.
                        */
                        if(USER_PORT_CHECK_I2C_MASTER_CMD(USER_PORT_I2C_MASTER_CMD_M_START_ON_IDLE))
                        {
                            USER_PORT_I2C_MASTER_CLEAR_START;

                            endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER; /* Pass control to Slave */
                        }
                        /* Valid arbitration lost on the address phase happens only when: master LOST_ARB is set and
                        * slave BUS_ERR is cleared. Only in that case set the state to IDLE without SCB IP re-enable.
                        */
                        else if((!USER_PORT_CHECK_INTR_SLAVE_MASKED(USER_PORT_INTR_SLAVE_I2C_BUS_ERROR))
                               && USER_PORT_CHECK_INTR_MASTER_MASKED(USER_PORT_INTR_MASTER_I2C_ARB_LOST))
                        {
                            endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER; /* Pass control to Slave */
                        }
                        else
                        {
                            endTransfer = 0u; /* Causes I2C_FSM_EXIT_IDLE to be set below */
                        }

                        if(0u != endTransfer) /* Clean-up master to proceed with slave */
                        {
                            USER_PORT_CLEAR_TX_FIFO; /* Shifter keeps address, clear it */

                            USER_PORT_DISABLE_MASTER_AUTO_DATA_ACK; /* In case of reading disable autoACK */

                            /* Clean-up master interrupt sources */
                            USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_ALL);

                            /* Disable data processing interrupts: they have to be cleared before */
                            USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
                            USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);

                            USER_PORT_state = USER_PORT_I2C_FSM_IDLE;
                        }
                        else
                        {
                            /* Set I2C_FSM_EXIT_IDLE for BUS_ERR and ARB_LOST (that is really bus error) */
                            USER_PORT_state = USER_PORT_I2C_FSM_EXIT_IDLE;
                        }
                    }
                    else
                    {
                        /* Set I2C_FSM_EXIT_IDLE if any other state than address */
                        USER_PORT_state = USER_PORT_I2C_FSM_EXIT_IDLE;
                    }
                }
                else
                {
                    /* In case of LOST*/
                    USER_PORT_state = USER_PORT_I2C_FSM_EXIT_IDLE;
                }
                
            }
        }
        
    }
    else /* (USER_PORT_CHECK_I2C_FSM_SLAVE) */
    {
        if(USER_PORT_I2C_SLAVE)
        {
            /* INTR_SLAVE_I2C_BUS_ERROR or USER_PORT_INTR_SLAVE_I2C_ARB_LOST:
            * A Misplaced Start or Stop condition occurred on the bus: set a flag
            * to notify an error condition.
            */
            if(USER_PORT_CHECK_INTR_SLAVE_MASKED(USER_PORT_INTR_SLAVE_I2C_BUS_ERROR |
                                                        USER_PORT_INTR_SLAVE_I2C_ARB_LOST))
            {
                if(USER_PORT_CHECK_I2C_FSM_RD)
                {
                    /* TX direction: master reads from slave */
                    USER_PORT_slStatus &= (uint8) ~USER_PORT_I2C_SSTAT_RD_BUSY;
                    USER_PORT_slStatus |= (uint8) (USER_PORT_I2C_SSTAT_RD_ERR |
                                                          USER_PORT_I2C_SSTAT_RD_CMPLT);
                }
                else
                {
                    /* RX direction: master writes into slave */
                    USER_PORT_slStatus &= (uint8) ~USER_PORT_I2C_SSTAT_WR_BUSY;
                    USER_PORT_slStatus |= (uint8) (USER_PORT_I2C_SSTAT_WR_ERR |
                                                          USER_PORT_I2C_SSTAT_WR_CMPLT);
                }

                USER_PORT_state = USER_PORT_I2C_FSM_EXIT_IDLE;
            }
        }
        
    }

    /* States description:
    * Any Master operation starts from: the ADDR_RD/WR state as the master generates traffic on the bus.
    * Any Slave operation starts from: the IDLE state as the slave always waits for actions from the master.
    */

    /* FSM Master */
    if(USER_PORT_CHECK_I2C_FSM_MASTER)
    {
        if(USER_PORT_I2C_MASTER)
        {
            /* INTR_MASTER_I2C_STOP:
            * A Stop condition was generated by the master: the end of the transaction.
            * Set completion flags to notify the API.
            */
            if(USER_PORT_CHECK_INTR_MASTER_MASKED(USER_PORT_INTR_MASTER_I2C_STOP))
            {
                USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_I2C_STOP);

                USER_PORT_mstrStatus |= (uint16) USER_PORT_GET_I2C_MSTAT_CMPLT;
                USER_PORT_state       = USER_PORT_I2C_FSM_IDLE;
            }
            else
            {
                if(USER_PORT_CHECK_I2C_FSM_ADDR) /* Address stage */
                {
                    /* INTR_MASTER_I2C_NACK:
                    * The master sent an address but it was NACKed by the slave. Complete transaction.
                    */
                    if(USER_PORT_CHECK_INTR_MASTER_MASKED(USER_PORT_INTR_MASTER_I2C_NACK))
                    {
                        USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_I2C_NACK);

                        USER_PORT_mstrStatus |= (uint16) (USER_PORT_I2C_MSTAT_ERR_XFER |
                                                                 USER_PORT_I2C_MSTAT_ERR_ADDR_NAK);

                        endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                    }
                    /* INTR_TX_UNDERFLOW. The master sent an address:
                    *  - TX direction: the clock is stretched after the ACK phase, because the TX FIFO is
                    *    EMPTY. The TX EMPTY cleans all the TX interrupt sources.
                    *  - RX direction: the 1st byte is received, but there is no ACK permission,
                    *    the clock is stretched after 1 byte is received.
                    */
                    else
                    {
                        if(USER_PORT_CHECK_I2C_FSM_RD) /* Reading */
                        {
                            USER_PORT_state = USER_PORT_I2C_FSM_MSTR_RD_DATA;
                        }
                        else /* Writing */
                        {
                            USER_PORT_state = USER_PORT_I2C_FSM_MSTR_WR_DATA;
                            if(0u != USER_PORT_mstrWrBufSize)
                            {
                                /* Enable INTR.TX_EMPTY if there is data to transmit */
                                USER_PORT_SetTxInterruptMode(USER_PORT_INTR_TX_EMPTY);
                            }
                        }
                    }
                }

                if(USER_PORT_CHECK_I2C_FSM_DATA) /* Data phase */
                {
                    if(USER_PORT_CHECK_I2C_FSM_RD) /* Reading */
                    {
                        /* INTR_RX_FULL:
                        * RX direction: the master received 8 bytes.
                        * Get data from RX FIFO and decide whether to ACK or  NACK the following bytes.
                        */
                        if(USER_PORT_CHECK_INTR_RX_MASKED(USER_PORT_INTR_RX_FULL))
                        {
                            /* Calculate difference */
                            diffCount =  USER_PORT_mstrRdBufSize -
                                        (USER_PORT_mstrRdBufIndex + USER_PORT_GET_RX_FIFO_ENTRIES);

                            /* Proceed transaction or end it when RX FIFO becomes FULL again */
                            if(diffCount > USER_PORT_I2C_FIFO_SIZE)
                            {
                                diffCount = USER_PORT_I2C_FIFO_SIZE;
                            }
                            else
                            {
                                if(0u == diffCount)
                                {
                                    USER_PORT_DISABLE_MASTER_AUTO_DATA_ACK;

                                    diffCount   = USER_PORT_I2C_FIFO_SIZE;
                                    endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                                }
                            }

                            for(; (0u != diffCount); diffCount--)
                            {
                                USER_PORT_mstrRdBufPtr[USER_PORT_mstrRdBufIndex] = (uint8)
                                                                                        USER_PORT_RX_FIFO_RD_REG;
                                USER_PORT_mstrRdBufIndex++;
                            }
                        }
                        /* INTR_RX_NOT_EMPTY:
                        * RX direction: the master received one data byte, ACK or NACK it.
                        * The last byte is stored and NACKed by the master. The NACK and Stop is
                        * generated by one command generate Stop.
                        */
                        else if(USER_PORT_CHECK_INTR_RX_MASKED(USER_PORT_INTR_RX_NOT_EMPTY))
                        {
                            /* Put data in component buffer */
                            USER_PORT_mstrRdBufPtr[USER_PORT_mstrRdBufIndex] = (uint8) USER_PORT_RX_FIFO_RD_REG;
                            USER_PORT_mstrRdBufIndex++;

                            if(USER_PORT_mstrRdBufIndex < USER_PORT_mstrRdBufSize)
                            {
                                USER_PORT_I2C_MASTER_GENERATE_ACK;
                            }
                            else
                            {
                               endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                            }
                        }
                        else
                        {
                            /* Do nothing */
                        }

                        USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_RX_ALL);
                    }
                    else /* Writing */
                    {
                        /* INTR_MASTER_I2C_NACK :
                        * The master writes data to the slave and NACK was received: not all the bytes were
                        * written to the slave from the TX FIFO. Revert the index if there is data in
                        * the TX FIFO and pass control to a complete transfer.
                        */
                        if(USER_PORT_CHECK_INTR_MASTER_MASKED(USER_PORT_INTR_MASTER_I2C_NACK))
                        {
                            USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_I2C_NACK);

                            /* Rollback write buffer index: NACKed byte remains in shifter */
                            USER_PORT_mstrWrBufIndexTmp -= (USER_PORT_GET_TX_FIFO_ENTRIES +
                                                                   USER_PORT_GET_TX_FIFO_SR_VALID);

                            /* Update number of transferred bytes */
                            USER_PORT_mstrWrBufIndex = USER_PORT_mstrWrBufIndexTmp;

                            USER_PORT_mstrStatus |= (uint16) (USER_PORT_I2C_MSTAT_ERR_XFER |
                                                                     USER_PORT_I2C_MSTAT_ERR_SHORT_XFER);

                            USER_PORT_CLEAR_TX_FIFO;

                            endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                        }
                        /* INTR_TX_EMPTY :
                        * TX direction: the TX FIFO is EMPTY, the data from the buffer needs to be put there.
                        * When there is no data in the component buffer, the underflow interrupt is
                        * enabled to catch when all the data has been transferred.
                        */
                        else if(USER_PORT_CHECK_INTR_TX_MASKED(USER_PORT_INTR_TX_EMPTY))
                        {
                            while(USER_PORT_I2C_FIFO_SIZE != USER_PORT_GET_TX_FIFO_ENTRIES)
                            {
                                /* The temporary mstrWrBufIndexTmp is used because slave could NACK the byte and index
                                * roll-back required in this case. The mstrWrBufIndex is updated at the end of transfer.
                                */
                                if(USER_PORT_mstrWrBufIndexTmp < USER_PORT_mstrWrBufSize)
                                {
                                #if(!USER_PORT_CY_SCBIP_V0)
                                   /* Clear INTR_TX.UNDERFLOW before putting the last byte into TX FIFO. This ensures
                                    * a proper trigger at the end of transaction when INTR_TX.UNDERFLOW single trigger
                                    * event. Ticket ID# 156735.
                                    */
                                    if(USER_PORT_mstrWrBufIndexTmp == (USER_PORT_mstrWrBufSize - 1u))
                                    {
                                        USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_TX_UNDERFLOW);
                                        USER_PORT_SetTxInterruptMode(USER_PORT_INTR_TX_UNDERFLOW);
                                    }
                                 #endif /* (!USER_PORT_CY_SCBIP_V0) */

                                    /* Put data into TX FIFO */
                                    USER_PORT_TX_FIFO_WR_REG = (uint32) USER_PORT_mstrWrBufPtr[USER_PORT_mstrWrBufIndexTmp];
                                    USER_PORT_mstrWrBufIndexTmp++;
                                }
                                else
                                {
                                    break; /* No more data to put */
                                }
                            }

                        #if(USER_PORT_CY_SCBIP_V0)
                            if(USER_PORT_mstrWrBufIndexTmp == USER_PORT_mstrWrBufSize)
                            {
                                USER_PORT_SetTxInterruptMode(USER_PORT_INTR_TX_UNDERFLOW);
                            }

                            USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_TX_ALL);
                        #else
                            USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_TX_EMPTY);
                        #endif /* (USER_PORT_CY_SCBIP_V0) */
                        }
                        /* INTR_TX_UNDERFLOW:
                        * TX direction: all data from the TX FIFO was transferred to the slave.
                        * The transaction needs to be completed.
                        */
                        else if(USER_PORT_CHECK_INTR_TX_MASKED(USER_PORT_INTR_TX_UNDERFLOW))
                        {
                            /* Update number of transferred bytes */
                            USER_PORT_mstrWrBufIndex = USER_PORT_mstrWrBufIndexTmp;

                            endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                        }
                        else
                        {
                            /* Do nothing */
                        }
                    }
                }

                if(0u != endTransfer) /* Complete transfer */
                {
                    /* Clean-up master after reading: only in case of NACK */
                    USER_PORT_DISABLE_MASTER_AUTO_DATA_ACK;

                    /* Disable data processing interrupts: they have to be cleared before */
                    USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
                    USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);

                    if(USER_PORT_CHECK_I2C_MODE_NO_STOP(USER_PORT_mstrControl))
                    {
                        /* On-going transaction is suspended: the ReStart is generated by the API request */
                        USER_PORT_mstrStatus |= (uint16) (USER_PORT_I2C_MSTAT_XFER_HALT |
                                                                 USER_PORT_GET_I2C_MSTAT_CMPLT);

                        USER_PORT_state = USER_PORT_I2C_FSM_MSTR_HALT;
                    }
                    else
                    {
                        /* Complete transaction: exclude the data processing state and generate Stop.
                        * The completion status will be set after Stop generation.
                        * A special case is read: because NACK and Stop are generated by the command below.
                        * Lost arbitration can occur during NACK generation when
                        * the other master is still reading from the slave.
                        */
                        USER_PORT_I2C_MASTER_GENERATE_STOP;
                    }
                }
            }

        } /* (USER_PORT_I2C_MASTER) */
        

    } /* (USER_PORT_CHECK_I2C_FSM_MASTER) */


    /* FSM Slave */
    else if(USER_PORT_CHECK_I2C_FSM_SLAVE)
    {
        if(USER_PORT_I2C_SLAVE)
        {
            /* INTR_SLAVE_NACK:
            * The master completes reading the slave: the appropriate flags have to be set.
            * The TX FIFO is cleared after an overflow condition is set.
            */
            if(USER_PORT_CHECK_INTR_SLAVE_MASKED(USER_PORT_INTR_SLAVE_I2C_NACK))
            {
                USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_I2C_NACK);

                /* All entries that remain in TX FIFO max value is 9: 8 (FIFO) + 1 (SHIFTER) */
                diffCount = (USER_PORT_GET_TX_FIFO_ENTRIES + USER_PORT_GET_TX_FIFO_SR_VALID);

                if(USER_PORT_slOverFlowCount > diffCount) /* Overflow */
                {
                    USER_PORT_slStatus |= (uint8) USER_PORT_I2C_SSTAT_RD_OVFL;
                }
                else /* No Overflow */
                {
                    /* Roll-back temporary index */
                    USER_PORT_slRdBufIndexTmp -= (diffCount - USER_PORT_slOverFlowCount);
                }

                /* Update slave of transferred bytes */
                USER_PORT_slRdBufIndex = USER_PORT_slRdBufIndexTmp;

                /* Clean-up TX FIFO */
                USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);
                USER_PORT_slOverFlowCount = 0u;
                USER_PORT_CLEAR_TX_FIFO;

                /* Complete master reading */
                USER_PORT_slStatus &= (uint8) ~USER_PORT_I2C_SSTAT_RD_BUSY;
                USER_PORT_slStatus |= (uint8)  USER_PORT_I2C_SSTAT_RD_CMPLT;
                USER_PORT_state     =  USER_PORT_I2C_FSM_IDLE;
            }


            /* INTR_SLAVE_I2C_WRITE_STOP:
            * The master completes writing to the slave: the appropriate flags have to be set.
            * The RX FIFO contains 1-8 bytes from the previous transaction which needs to be read.
            * There is a possibility that RX FIFO contains an address, it needs to leave it there.
            */
            if(USER_PORT_CHECK_INTR_SLAVE_MASKED(USER_PORT_INTR_SLAVE_I2C_WRITE_STOP))
            {
                USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_I2C_WRITE_STOP);

                /* Read bytes from RX FIFO when auto data ACK receive logic is enabled. Otherwise all data bytes
                * were already read from the RX FIFO except for address byte which has to stay here to be handled by
                * I2C_ADDR_MATCH.
                */
                if (0u != (USER_PORT_I2C_CTRL_REG & USER_PORT_I2C_CTRL_S_READY_DATA_ACK))
                {
                    while(0u != USER_PORT_GET_RX_FIFO_ENTRIES)
                    {
                        if(USER_PORT_CHECK_I2C_ACCEPT_ADDRESS)
                        {
                            if((1u == USER_PORT_GET_RX_FIFO_ENTRIES) &&
                               (USER_PORT_CHECK_INTR_SLAVE_MASKED(USER_PORT_INTR_SLAVE_I2C_ADDR_MATCH)))
                            {
                                break; /* Leave address in RX FIFO */
                            }
                        }
                        

                        /* Put data in component buffer */
                        USER_PORT_slWrBufPtr[USER_PORT_slWrBufIndex] = (uint8) USER_PORT_RX_FIFO_RD_REG;
                        USER_PORT_slWrBufIndex++;
                    }

                    USER_PORT_DISABLE_SLAVE_AUTO_DATA;
                }

                if(USER_PORT_CHECK_INTR_RX(USER_PORT_INTR_RX_OVERFLOW))
                {
                    USER_PORT_slStatus |= (uint8) USER_PORT_I2C_SSTAT_WR_OVFL;
                }

                /* Clears RX interrupt sources triggered on data receiving */
                USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
                USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_RX_ALL);

                /* Complete master writing */
                USER_PORT_slStatus &= (uint8) ~USER_PORT_I2C_SSTAT_WR_BUSY;
                USER_PORT_slStatus |= (uint8)  USER_PORT_I2C_SSTAT_WR_CMPLT;
                USER_PORT_state     =  USER_PORT_I2C_FSM_IDLE;
            }


            /* INTR_SLAVE_I2C_ADDR_MATCH:
            * The address match event starts the slave operation: after leaving the TX or RX
            * direction has to be chosen.
            * The wakeup interrupt must be cleared only after an address match is set.
            */
            if(USER_PORT_CHECK_INTR_SLAVE_MASKED(USER_PORT_INTR_SLAVE_I2C_ADDR_MATCH))
            {
                if(USER_PORT_CHECK_I2C_ACCEPT_ADDRESS)
                {
                    address = USER_PORT_RX_FIFO_RD_REG; /* Address in the RX FIFO */

                    /* Clears RX sources if address was received in RX FIFO */
                    USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_RX_ALL);

                    if(0u != address)
                    {
                        /* Suppress compiler warning */
                    }
                }
                

                if(USER_PORT_CHECK_I2C_STATUS(USER_PORT_I2C_STATUS_S_READ))
                /* TX direction: master reads from slave */
                {
                    USER_PORT_SetTxInterruptMode(USER_PORT_INTR_TX_EMPTY);

                    /* Set temporary index to address buffer clear from API */
                    USER_PORT_slRdBufIndexTmp = USER_PORT_slRdBufIndex;

                    /* Start master reading */
                    USER_PORT_slStatus |= (uint8) USER_PORT_I2C_SSTAT_RD_BUSY;
                    USER_PORT_state     = USER_PORT_I2C_FSM_SL_RD;
                }
                else
                /* RX direction: master writes into slave */
                {
                    /* Calculate available buffer size */
                    diffCount = (USER_PORT_slWrBufSize - USER_PORT_slWrBufIndex);

                #if (USER_PORT_CY_SCBIP_V0)

                    if(diffCount < USER_PORT_I2C_FIFO_SIZE)
                    /* Receive data: byte-by-byte */
                    {
                        USER_PORT_SetRxInterruptMode(USER_PORT_INTR_RX_NOT_EMPTY);
                    }
                    else
                    /* Receive data: into RX FIFO */
                    {
                        if(diffCount == USER_PORT_I2C_FIFO_SIZE)
                        {
                            /* NACK when RX FIFO become FULL */
                            USER_PORT_ENABLE_SLAVE_AUTO_DATA;
                        }
                        else
                        {
                            /* Stretch clock when RX FIFO becomes FULL */
                            USER_PORT_ENABLE_SLAVE_AUTO_DATA_ACK;
                            USER_PORT_SetRxInterruptMode(USER_PORT_INTR_RX_FULL);
                        }
                    }

                #else

                    if(USER_PORT_CHECK_I2C_ACCEPT_ADDRESS)
                    {
                        /* Enable RX.NOT_EMPTY interrupt source to receive byte by byte.
                        * The byte by byte receive is always chosen for the case when an address is accepted in RX FIFO.
                        * Ticket ID#175559.
                        */
                        USER_PORT_SetRxInterruptMode(USER_PORT_INTR_RX_NOT_EMPTY);
                    }
                    else
                    {
                        if(diffCount < USER_PORT_I2C_FIFO_SIZE)
                        /* Receive data: byte-by-byte */
                        {
                            USER_PORT_SetRxInterruptMode(USER_PORT_INTR_RX_NOT_EMPTY);
                        }
                        else
                        /* Receive data: into RX FIFO */
                        {
                            if(diffCount == USER_PORT_I2C_FIFO_SIZE)
                            {
                                /* NACK when RX FIFO become FULL */
                                USER_PORT_ENABLE_SLAVE_AUTO_DATA;
                            }
                            else
                            {
                                /* Stretch clock when RX FIFO becomes FULL */
                                USER_PORT_ENABLE_SLAVE_AUTO_DATA_ACK;
                                USER_PORT_SetRxInterruptMode(USER_PORT_INTR_RX_FULL);
                            }
                        }
                    }
                    

                #endif /* (USER_PORT_CY_SCBIP_V0) */

                    /* Start master reading */
                    USER_PORT_slStatus |= (uint8) USER_PORT_I2C_SSTAT_WR_BUSY;
                    USER_PORT_state     = USER_PORT_I2C_FSM_SL_WR;
                }

                /* Clear interrupts before ACK address */
                USER_PORT_ClearI2CExtClkInterruptSource(USER_PORT_INTR_I2C_EC_WAKE_UP);
                USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_ALL);

                /* Preparation complete: ACK the address */
                USER_PORT_I2C_SLAVE_GENERATE_ACK;
            }

            /* USER_PORT_INTR_RX_FULL:
            * Get data from the RX FIFO and decide whether to ACK or NACK the following bytes
            */
            if(USER_PORT_CHECK_INTR_RX_MASKED(USER_PORT_INTR_RX_FULL))
            {
                /* Calculate available buffer size to take into account that RX FIFO is FULL */
                diffCount =  USER_PORT_slWrBufSize -
                            (USER_PORT_slWrBufIndex + USER_PORT_I2C_FIFO_SIZE);

                if(diffCount > USER_PORT_I2C_FIFO_SIZE) /* Proceed transaction */
                {
                    diffCount   = USER_PORT_I2C_FIFO_SIZE;
                    endTransfer = 0u;  /* Continue active transfer */
                }
                else /* End when FIFO becomes FULL again */
                {
                    endTransfer = USER_PORT_I2C_CMPLT_ANY_TRANSFER;
                }

                for(; (0u != diffCount); diffCount--)
                {
                    /* Put data in component buffer */
                    USER_PORT_slWrBufPtr[USER_PORT_slWrBufIndex] = (uint8) USER_PORT_RX_FIFO_RD_REG;
                    USER_PORT_slWrBufIndex++;
                }

                if(0u != endTransfer) /* End transfer sending NACK */
                {
                    USER_PORT_ENABLE_SLAVE_AUTO_DATA_NACK;

                    /* INTR_RX_FULL triggers earlier than INTR_SLAVE_I2C_STOP:
                    * disable all RX interrupt sources.
                    */
                    USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
                }

                USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_RX_FULL);
            }
            /* USER_PORT_INTR_RX_NOT_EMPTY:
            * The buffer size is less than 8: it requires processing in byte-by-byte mode.
            */
            else if(USER_PORT_CHECK_INTR_RX_MASKED(USER_PORT_INTR_RX_NOT_EMPTY))
            {
                diffCount = USER_PORT_RX_FIFO_RD_REG;

                if(USER_PORT_slWrBufIndex < USER_PORT_slWrBufSize)
                {
                    USER_PORT_I2C_SLAVE_GENERATE_ACK;

                    /* Put data into component buffer */
                    USER_PORT_slWrBufPtr[USER_PORT_slWrBufIndex] = (uint8) diffCount;
                    USER_PORT_slWrBufIndex++;
                }
                else /* Overflow: there is no space in write buffer */
                {
                    USER_PORT_I2C_SLAVE_GENERATE_NACK;

                    USER_PORT_slStatus |= (uint8) USER_PORT_I2C_SSTAT_WR_OVFL;
                }

                USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_RX_NOT_EMPTY);
            }
            else
            {
                /* Does nothing */
            }


            /* USER_PORT_INTR_TX_EMPTY:
            * The master reads the slave: provide data to read or 0xFF in the case of the end of the buffer
            * The overflow condition must be captured, but not set until the end of transaction.
            * There is a possibility of a false overflow due to TX FIFO utilization.
            */
            if(USER_PORT_CHECK_INTR_TX_MASKED(USER_PORT_INTR_TX_EMPTY))
            {
                while(USER_PORT_I2C_FIFO_SIZE != USER_PORT_GET_TX_FIFO_ENTRIES)
                {
                    /* Temporary slRdBufIndexTmp is used because the master can NACK the byte and
                    * index roll-back is required in this case. The slRdBufIndex is updated at the end
                    * of the read transfer.
                    */
                    if(USER_PORT_slRdBufIndexTmp < USER_PORT_slRdBufSize)
                    /* Data from buffer */
                    {
                        USER_PORT_TX_FIFO_WR_REG = (uint32) USER_PORT_slRdBufPtr[USER_PORT_slRdBufIndexTmp];
                        USER_PORT_slRdBufIndexTmp++;
                    }
                    else
                    /* Probably Overflow */
                    {
                        USER_PORT_TX_FIFO_WR_REG = USER_PORT_I2C_SLAVE_OVFL_RETURN;

                        if(0u == (USER_PORT_INTR_TX_OVERFLOW & USER_PORT_slOverFlowCount))
                        {
                            /* Get counter in range of byte: value 10 is overflow */
                            USER_PORT_slOverFlowCount++;
                        }
                    }
                }

                USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_TX_EMPTY);
            }

        }  /* (USER_PORT_I2C_SLAVE) */
        
    }


    /* FSM EXIT:
    * Slave:  INTR_SLAVE_I2C_BUS_ERROR, INTR_SLAVE_I2C_ARB_LOST
    * Master: INTR_MASTER_I2C_BUS_ERROR, INTR_MASTER_I2C_ARB_LOST.
    */
    else
    {
        USER_PORT_CTRL_REG &= (uint32) ~USER_PORT_CTRL_ENABLED; /* Disable scb IP */

        USER_PORT_state = USER_PORT_I2C_FSM_IDLE;

        USER_PORT_DISABLE_SLAVE_AUTO_DATA;
        USER_PORT_DISABLE_MASTER_AUTO_DATA;

    #if(USER_PORT_CY_SCBIP_V0)
        USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);

        /* Clear interrupt sources as they are not automatically cleared after SCB is disabled */
        USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_RX_ALL);
        USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_TX_ALL);
        USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_ALL);
        USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_ALL);
    #endif /* (USER_PORT_CY_SCBIP_V0) */

        USER_PORT_CTRL_REG |= (uint32) USER_PORT_CTRL_ENABLED;  /* Enable scb IP */
    }
}


/* [] END OF FILE */
