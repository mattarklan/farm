from __future__ import annotations

from typing import Optional

from farm.core.api_client import IngestionAPIClient


class TrendClient:

    def __init__(
        self,
        api_client: IngestionAPIClient
    ):

        self.api_client = api_client


    def get_hourly_metric(
        self,
        asset_id: str,
        metric: str,
        since_ts: int = 0
    ) -> list[dict]:

        rows = self.api_client.get_hourly_trends(
            asset_id,
            since_ts
        )

        points = []

        for row in rows:

            if row.get("metric") != metric:
                continue


            value = row.get("avg")

            if value is None:
                continue


            points.append(
                {
                    "timestamp": row.get(
                        "hour"
                    ),

                    "value": float(
                        value
                    ),

                    "minimum": row.get(
                        "min"
                    ),

                    "maximum": row.get(
                        "max"
                    ),

                    "count": row.get(
                        "count"
                    ),
                }
            )


        points.sort(
            key=lambda p: (
                p["timestamp"]
                or 0
            )
        )


        return points



    def get_daily_metric(
        self,
        asset_id: str,
        metric: str,
        since_ts: int = 0
    ) -> list[dict]:

        rows = self.api_client.get_daily_trends(
            asset_id,
            since_ts
        )

        points = []

        for row in rows:

            if row.get("metric") != metric:
                continue


            value = row.get("avg")

            if value is None:
                continue


            points.append(
                {
                    "timestamp": row.get(
                        "day"
                    ),

                    "value": float(
                        value
                    ),

                    "minimum": row.get(
                        "min"
                    ),

                    "maximum": row.get(
                        "max"
                    ),

                    "count": row.get(
                        "count"
                    ),
                }
            )


        points.sort(
            key=lambda p: (
                p["timestamp"]
                or 0
            )
        )


        return points