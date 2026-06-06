package routing

import (
	"bufio"
	"context"
	"fmt"
	"log"
	"os"
	"time"
)

func (crp *CRPRoutingEngine) InitBackgroundWorker(ctx context.Context) {
	go crp.checkCustomizerUpdate(crp.metrics.GetFilePath(), ctx)
	// go crp.checkActivatedConditionalRestrictions(ctx) // belum jadi
}

func (crp *CRPRoutingEngine) checkCustomizerUpdate(metricsFilePath string, ctx context.Context) {
	lastModifiedTime, err := isFileUpdated(metricsFilePath)
	if err != nil {
		crp.logger.Sugar().Warnf("engine.checkCustomizerUpdate: failed to read file modification time : %v\n", err)
	}

	metricsReaderTimer := time.NewTicker(CUSTOMIZER_UPDATER_TIMER_SECONDS)
	defer metricsReaderTimer.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-metricsReaderTimer.C:
			currModifiedTime, err := isFileUpdated(metricsFilePath)
			if err != nil {
				crp.logger.Sugar().Warnf("engine.checkCustomizerUpdate: failed to read file modification time: %v\n", err)
				continue
			}

			if currModifiedTime != lastModifiedTime {
				crp.logger.Sugar().Infof("engine.checkCustomizerUpdate: file modification time changed  old=%d  new=%d\n, updating the metrics and costFunction....", lastModifiedTime, currModifiedTime)
				err := crp.updateMetrics()
				if err != nil {
					success := false
					for attempts := 0; attempts < MAX_RETRY; attempts++ {
						// exponential backoff: 1s, 2s, 4s, 8s, ...
						delay := time.Duration(1<<attempts) * time.Second

						log.Printf("Scheduling retry for updating metrics job in %v", delay)
						delaytt := time.NewTimer(delay)
						select {
						case <-delaytt.C:
							select {
							case <-ctx.Done():
								return
							default:
								err := crp.updateMetrics()
								if err != nil {
									continue
								} else {
									success = true
									break
								}
							}
						case <-ctx.Done():
							return
						}
						delaytt.Stop()
					}
					if !success {
						crp.logger.Sugar().Errorf("engine.checkCustomizerUpdate: update metrics job (modification time: %v) failed permanently: %v", currModifiedTime, err)
					} else {
						lastModifiedTime = currModifiedTime
					}
				} else {
					lastModifiedTime = currModifiedTime
				}
			}

		}
	}
}

func (crp *CRPRoutingEngine) updateMetrics() (err error) {
	defer func() {
		if r := recover(); r != nil {
			err = fmt.Errorf("panic recovered: %v", r)
		}
	}()
	readBuf := bufio.NewReaderSize(nil, 4096*4)
	err = crp.metrics.UpdateMetrics(readBuf)
	if err != nil {
		crp.logger.Sugar().Errorf("engine.checkCustomizerUpdate: failed to update metrics data: %v\n", err)
		return err
	}
	err = crp.lm.UpdateLandmarks(crp.landmarkFile, readBuf)
	if err != nil {
		crp.logger.Sugar().Errorf("engine.checkCustomizerUpdate: failed to update precalculated landmark distances: %v\n", err)
		return err
	}
	crp.puCache.Clear()
	crp.logger.Sugar().Infof("updated the metrics and costFunction....")

	return nil
}

func isFileUpdated(path string) (int64, error) {

	info, err := os.Stat(path)
	if err != nil {
		return 0, fmt.Errorf("isFileUpdated: failed to os.Stat file: %s", path)
	}
	lastModTime := info.ModTime().Unix()
	return lastModTime, nil
}

// func (crp *CRPRoutingEngine) checkActivatedConditionalRestrictions(ctx context.Context) {

// 	conditionalRestrCheckerTimer := time.NewTicker(CONDITIONAL_RESTRICTION_TIMER_SECONDS)
// 	defer conditionalRestrCheckerTimer.Stop()

// 	for {
// 		select {
// 		case <-ctx.Done():
// 			return
// 		case <-conditionalRestrCheckerTimer.C:
// 			now := time.Now()
// 			err := crp.activateConditionalRestrictions(now) // bentar belum jadi
// 			if err != nil {
// 				success := false
// 				for attempts := 0; attempts < MAX_RETRY; attempts++ {
// 					// exponential backoff: 1s, 2s, 4s, 8s, ...
// 					delay := time.Duration(1<<attempts) * time.Second

// 					log.Printf("Scheduling retry for updating metrics job in %v", delay)
// 					delaytt := time.NewTimer(delay)
// 					select {
// 					case <-delaytt.C:
// 						select {
// 						case <-ctx.Done():
// 							return
// 						default:
// 							err = crp.activateConditionalRestrictions(now) // bentar belum jadi
// 							if err != nil {
// 								continue
// 							} else {
// 								success = true
// 								break
// 							}
// 						}
// 					case <-ctx.Done():
// 						return
// 					}
// 					delaytt.Stop()
// 				}
// 				if !success {
// 					crp.logger.Sugar().Errorf("engine.checkCustomizerUpdate: update metrics job (modification time: %v) failed permanently: %v", now, err)
// 				}
// 			}
// 		}
// 	}
// }

// type edgeSpeedLimit struct {
// 	eId   da.Index
// 	speed float64
// }

// func newEdgeSpeedLimit(eId da.Index, speed float64) edgeSpeedLimit {
// 	return edgeSpeedLimit{eId: eId, speed: speed}
// }

// func (crp *CRPRoutingEngine) activateConditionalRestrictions(now time.Time) error {
// 	activatedBarrierVIds := make([]da.Index, 0)
// 	crp.graph.ForEachConditionalBarrierNode(func(id da.Index, res da.ConditionalBarrierNode) {
// 		prohibited, err := op.IsConditionalRestrictionCurrentlyProhibited(now, res.GetTimeRangeVal())
// 		if err != nil {
// 			crp.logger.Sugar().Warnf("activateConditionalRestrictions: failed to check barrier node conditional restriction %s", err.Error())
// 		}
// 		activated := !prohibited
// 		if activated {
// 			vId := crp.verticesLookupTable.Get(uint64(res.GetOsmNodeId()))
// 			activatedBarrierVIds = append(activatedBarrierVIds, da.Index(vId))
// 		}
// 	})

// 	activatedReversibleEdgeIds := make([]da.Index, 0)
// 	crp.graph.ForEachConditionalReversibleEdge(func(id da.Index, res da.ConditionalReversibleEdge) {
// 		// https://wiki.openstreetmap.org/wiki/Tag:oneway%3Dreversible
// 		eId := res.GetEdgeId()
// 		prohibited, err := op.IsConditionalRestrictionCurrentlyProhibited(now, res.GetTimeRangeVal())
// 		if err != nil {
// 			crp.logger.Sugar().Warnf("activateConditionalRestrictions: failed to check barrier node conditional restriction %s", err.Error())
// 		}
// 		activated := !prohibited
// 		if activated {
// 			activatedReversibleEdgeIds = append(activatedReversibleEdgeIds, eId)
// 		}
// 	})

// 	activatedSpeedLimits := make([]edgeSpeedLimit, 0)
// 	crp.graph.ForEachConditionalSpeedLimit(func(id da.Index, res da.ConditionalSpeedLimit) {
// 		eId := res.GetEdgeId()
// 		speedStr := op.GetAccessValConditionalRestriction(res.GetTimeRangeSpeedVal())
// 		speed, err := util.ParseFloat64(speedStr)
// 		if err != nil {
// 			crp.logger.Sugar().Warnf("activateConditionalRestrictions: failed to parseFloat() edge speed %s", err.Error())
// 		}
// 		timeRange := op.GetTimeRangeValConditionalRestriction(res.GetTimeRangeSpeedVal())
// 		prohibited, err := op.IsConditionalRestrictionCurrentlyProhibited(now, timeRange)
// 		if err != nil {
// 			crp.logger.Sugar().Warnf("activateConditionalRestrictions: failed to check barrier node conditional restriction %s", err.Error())
// 		}
// 		activated := !prohibited
// 		if activated {
// 			activatedSpeedLimits = append(activatedSpeedLimits, newEdgeSpeedLimit(eId, speed))
// 		}
// 	})

// 	activatedVehicleNoAccessEdgeIds := make([]da.Index, 0)

// 	crp.graph.ForEachConditionalTrafficMode(func(id da.Index, res da.ConditionalTrafficMode) {
// 		eId := res.GetEdgeId()
// 		prohibited, err := op.IsConditionalRestrictionCurrentlyProhibited(now, res.GetTimeRangeVal())
// 		if err != nil {
// 			crp.logger.Sugar().Warnf("activateConditionalRestrictions: failed to check barrier node conditional restriction %s", err.Error())
// 		}
// 		activated := !prohibited
// 		if activated {
// 			activatedVehicleNoAccessEdgeIds = append(activatedVehicleNoAccessEdgeIds, eId)
// 		}
// 	})

// 	activatedCondTurnRests := make([]da.ConditionalTurnRestriction, 0)
// 	crp.graph.ForEachConditionalTurnRestriction(func(id da.Index, res da.ConditionalTurnRestriction) {
// 		prohibited, err := op.IsConditionalRestrictionCurrentlyProhibited(now, res.GetTimeRangeVal())
// 		if err != nil {
// 			crp.logger.Sugar().Warnf("activateConditionalRestrictions: failed to check barrier node conditional restriction %s", err.Error())
// 		}
// 		activated := !prohibited
// 		if activated {
// 			activatedCondTurnRests = append(activatedCondTurnRests, res)
// 		}
// 	})

// 	// lastSegmentSpeedFilePaths := crp.metrics.GetLastSegmentSpeedFiles()

// 	return nil
// }
