package routing

import (
	"context"
	"fmt"
	"log"
	"os"
	"time"
)

func (crp *CRPRoutingEngine) InitBackgroundWorker(ctx context.Context) {
	go crp.checkCustomizerUpdate(crp.metrics.GetFilePath(), ctx)
}

func (crp *CRPRoutingEngine) checkCustomizerUpdate(metricsFilePath string, ctx context.Context) {
	lastModifiedTime, err := isFileUpdated(metricsFilePath)
	if err != nil {
		crp.logger.Sugar().Warnf("engine.checkCustomizerUpdate: failed to read file modification time : %v\n", err)
	}

	timer := time.NewTicker(CUSTOMIZER_UPDATER_TIMER_SECONDS)
	defer timer.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-timer.C:
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
	err = crp.metrics.UpdateMetrics()
	if err != nil {
		crp.logger.Sugar().Errorf("engine.checkCustomizerUpdate: failed to update metrics data: %v\n", err)
		return err
	}
	err = crp.lm.UpdateLandmarks(crp.landmarkFile)
	if err != nil {
		crp.logger.Sugar().Errorf("engine.checkCustomizerUpdate: failed to update precalculated landmark distances: %v\n", err)
		return err
	}
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
