/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_DEVICE_H
#define __ASM_DEVICE_H

struct dma_iommu_mapping;

struct dev_archdata {
	struct dma_map_ops *dma_ops;
#ifdef CONFIG_IOMMU_API
	void *iommu;			/* private IOMMU data */
	struct dma_iommu_mapping *mapping;
#endif
	bool dma_coherent;
	bool dma_noncontig;
};

struct pdev_archdata {
};

#ifdef CONFIG_IOMMU_API
#define to_dma_iommu_mapping(dev) (dev->archdata.mapping)
#else
#define to_dma_iommu_mapping(dev) NULL
#endif

#endif
